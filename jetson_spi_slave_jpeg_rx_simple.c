#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <pwd.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DEVICE "/dev/spidev0.0"
#define DEFAULT_CHUNK_SIZE 250U
#define DEFAULT_MAX_SPEED 4000000U
#define DEFAULT_MAX_FRAME_SIZE (8U * 1024U * 1024U)
#define DEFAULT_GAP_WARN_LIMIT 20U
#define DEFAULT_READ_BATCH_PACKETS 16U
#define SPIDEV_DEFAULT_BUFSIZ_HINT 4096U
#define RX_QUEUE_DEPTH 256U
#define RX_THREAD_PRIORITY 80
#define STATUS_INTERVAL_SEC 1
#define SYNC_DIAG_LIMIT 20U
#define RX_BLOCK_DIAG_LIMIT 20U
#define GAP_WARN_MIN_PACKETS 10U

#define JPEG_SOI_0 0xFF
#define JPEG_SOI_1 0xD8
#define JPEG_EOI_0 0xFF
#define JPEG_EOI_1 0xD9

#define APP_FRAME_SIZE     250U
#define APP_HDR_SIZE       10U
#define APP_PAYLOAD_MAX    240U
#define APP_SYNC           0xAA
#define APP_TAIL           0xBB

static volatile sig_atomic_t g_stop = 0;

static const char *g_device = DEFAULT_DEVICE;
static uint32_t g_mode = 1;
static uint8_t g_bits = 8;
static uint32_t g_speed = DEFAULT_MAX_SPEED;
static uint16_t g_delay_usecs = 0;
static size_t g_chunk_size = DEFAULT_CHUNK_SIZE;
static size_t g_transfer_size = 0;
static size_t g_payload_offset = 0;
static size_t g_read_buffer_size = 0;
static size_t g_max_frame_size = DEFAULT_MAX_FRAME_SIZE;
static unsigned int g_max_images = 0;
static unsigned int g_dump_packets = 0;
static unsigned int g_gap_warn_limit = DEFAULT_GAP_WARN_LIMIT;
static int g_verbose = 0;
static int g_force_read = 1;
static int g_method_explicit = 0;
static int g_tx_dummy = 0;
static unsigned int g_gap_warnings_printed = 0;
static unsigned int g_sync_diag_printed = 0;
static unsigned int g_rx_block_diag_printed = 0;

struct jpeg_state {
    uint8_t *buf;
    size_t len;
    size_t cap;
    int capturing;
    int have_prev;
    uint8_t prev;
    int have_tail_prev;
    uint8_t tail_prev;
    unsigned int image_index;
};

struct stream_stats {
    unsigned long long bytes;
    unsigned long long nonzero;
    unsigned long long ff;
    unsigned long long soi;
    unsigned long long eoi;
    unsigned long long soi_lsb_first;
    unsigned long long eoi_lsb_first;
    int have_prev;
    uint8_t prev;
};

struct app_stats {
    unsigned long long packets;
    unsigned long long no_sync;
    unsigned long long bad_header;
    unsigned long long bad_len;
    unsigned long long offset_wraps;
    unsigned long long offset_gaps;
    unsigned long long offset_gap_bytes;
    unsigned long long offset_gap_packets;
    unsigned long long suppressed_gap_warnings;
    unsigned long long gap_exact_packets;
    unsigned long long gap_partial_packets;
    uint32_t last_gap_bytes;
    uint32_t last_gap_packets;
    unsigned long long sync_candidates;
    unsigned long long false_sync;
    unsigned long long short_reads;
    unsigned long long rx_blocks;
    unsigned long long aligned_headers;
    unsigned long long aligned_bad_headers;
    unsigned long long misaligned_headers;
    unsigned long long phase_headers;
    unsigned long long phase_bad_headers;
    size_t last_expected_phase;
    size_t last_rx_len;
    int have_offset;
    uint32_t last_offset;
    uint32_t expected_offset;
};

struct app_stream_state {
    uint8_t *buf;
    size_t len;
    size_t cap;
};

struct rx_queue {
    uint8_t *storage;
    size_t block_size;
    unsigned int depth;
    unsigned int *free_idx;
    unsigned int *ready_idx;
    size_t *lens;
    unsigned int free_head;
    unsigned int free_tail;
    unsigned int free_count;
    unsigned int ready_head;
    unsigned int ready_tail;
    unsigned int ready_count;
    int closed;
    pthread_mutex_t lock;
    pthread_cond_t can_read;
    pthread_cond_t can_write;
};

struct rx_reader_args {
    int fd;
    int using_read_method;
    size_t spi_transfer_count;
    struct rx_queue *queue;
};

static void on_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    puts("  -D, --device      SPI device node, default /dev/spidev0.0\n"
         "  -s, --speed       max speed hint in Hz, default 4000000; 0 leaves slave speed unset\n"
         "  -d, --delay       delay in usec for each transfer\n"
         "  -b, --bpw         bits per word, default 8\n"
         "  -g, --chunk       valid payload bytes per block, default 250\n"
         "  -x, --transfer    total bytes per userspace receive; ioctl mode splits it into chunk-sized SPI transfers\n"
         "  -P, --payload-off leading bytes to skip per transfer, default 0\n"
         "  -B, --rx-buffer   read() buffer size, default 16 packets / 4000 bytes for 250-byte packets\n"
         "  -p, --packets     dump first N SPI transfers as hex\n"
         "  -m, --max-frame   max JPEG buffer size, default 8MB\n"
         "  -n, --images      stop after N saved JPEGs, default unlimited\n"
         "  -W, --gap-warn-limit print at most N gap warnings, default 20, 0 disables\n"
         "  -H, --cpha        set CPHA\n"
         "  -O, --cpol        set CPOL\n"
         "  -L, --lsb         set LSB first\n"
         "  -C, --cs-high     chip select active high\n"
         "  -3, --3wire       SI/SO shared\n"
         "  -N, --no-cs       no chip select\n"
         "  -R, --ready       slave ready mode\n"
        "  -v, --verbose     print verbose log\n"
         "  -r, --read        force read() instead of SPI_IOC_MESSAGE\n"
         "  -I, --ioctl       force SPI_IOC_MESSAGE instead of read()\n"
         "  -T, --tx-dummy    provide a zero TX buffer for SPI_IOC_MESSAGE\n"
         "      app packets: AA image_size32 offset32 BB + 240-byte payload\n"
         "  -h, --help        show this help\n");
}

static void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            {"device", 1, 0, 'D'},
            {"speed", 1, 0, 's'},
            {"delay", 1, 0, 'd'},
            {"bpw", 1, 0, 'b'},
            {"chunk", 1, 0, 'g'},
            {"transfer", 1, 0, 'x'},
            {"payload-off", 1, 0, 'P'},
            {"rx-buffer", 1, 0, 'B'},
            {"packets", 1, 0, 'p'},
            {"max-frame", 1, 0, 'm'},
            {"images", 1, 0, 'n'},
            {"gap-warn-limit", 1, 0, 'W'},
            {"cpha", 0, 0, 'H'},
            {"cpol", 0, 0, 'O'},
            {"lsb", 0, 0, 'L'},
            {"cs-high", 0, 0, 'C'},
            {"3wire", 0, 0, '3'},
            {"no-cs", 0, 0, 'N'},
            {"ready", 0, 0, 'R'},
            {"verbose", 0, 0, 'v'},
            {"read", 0, 0, 'r'},
            {"ioctl", 0, 0, 'I'},
            {"tx-dummy", 0, 0, 'T'},
            {"help", 0, 0, 'h'},
            {NULL, 0, 0, 0},
        };

        int c = getopt_long(argc, argv, "D:s:d:b:g:x:P:B:p:m:n:W:HOLC3NRrvITh", lopts, NULL);
        if (c == -1) {
            break;
        }

        switch (c) {
        case 'D':
            g_device = optarg;
            break;
        case 's':
            g_speed = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'd':
            g_delay_usecs = (uint16_t)strtoul(optarg, NULL, 0);
            break;
        case 'b':
            g_bits = (uint8_t)strtoul(optarg, NULL, 0);
            break;
        case 'g':
            g_chunk_size = (size_t)strtoull(optarg, NULL, 0);
            break;
        case 'x':
            g_transfer_size = (size_t)strtoull(optarg, NULL, 0);
            break;
        case 'P':
            g_payload_offset = (size_t)strtoull(optarg, NULL, 0);
            break;
        case 'B':
            g_read_buffer_size = (size_t)strtoull(optarg, NULL, 0);
            break;
        case 'p':
            g_dump_packets = (unsigned int)strtoul(optarg, NULL, 0);
            break;
        case 'm':
            g_max_frame_size = (size_t)strtoull(optarg, NULL, 0);
            break;
        case 'n':
            g_max_images = (unsigned int)strtoul(optarg, NULL, 0);
            break;
        case 'W':
            g_gap_warn_limit = (unsigned int)strtoul(optarg, NULL, 0);
            break;
        case 'H':
            g_mode |= SPI_CPHA;
            break;
        case 'O':
            g_mode |= SPI_CPOL;
            break;
        case 'L':
            g_mode |= SPI_LSB_FIRST;
            break;
        case 'C':
            g_mode |= SPI_CS_HIGH;
            break;
        case '3':
            g_mode |= SPI_3WIRE;
            break;
        case 'N':
            g_mode |= SPI_NO_CS;
            break;
        case 'R':
            g_mode |= SPI_READY;
            break;
        case 'v':
            g_verbose = 1;
            break;
        case 'r':
            g_force_read = 1;
            g_method_explicit = 1;
            break;
        case 'I':
            g_force_read = 0;
            g_method_explicit = 1;
            break;
        case 'T':
            g_tx_dummy = 1;
            break;
        case 'h':
            print_usage(argv[0]);
            exit(EXIT_SUCCESS);
        default:
            print_usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    if (g_chunk_size == 0) {
        fprintf(stderr, "chunk size must be > 0\n");
        exit(EXIT_FAILURE);
    }
    if (g_transfer_size == 0) {
        g_transfer_size = g_payload_offset + g_chunk_size;
    }
    if (g_payload_offset >= g_transfer_size) {
        fprintf(stderr, "payload offset must be smaller than transfer size\n");
        exit(EXIT_FAILURE);
    }
    if (g_transfer_size - g_payload_offset < g_chunk_size) {
        fprintf(stderr, "transfer size minus payload offset must be >= chunk size\n");
        exit(EXIT_FAILURE);
    }
    {
        size_t spi_segment_size = g_payload_offset + g_chunk_size;
        if (spi_segment_size == 0 || (g_transfer_size % spi_segment_size) != 0) {
            fprintf(stderr,
                    "transfer size must be an integer multiple of payload offset + chunk size "
                    "(%zu); e.g. -g 250 -x 2500 -P 0\n",
                    spi_segment_size);
            exit(EXIT_FAILURE);
        }
    }
    if (g_payload_offset != 0 && g_transfer_size > (g_payload_offset + g_chunk_size)) {
        fprintf(stderr,
                "batched ioctl receive currently requires payload offset 0; use -P 0\n");
        exit(EXIT_FAILURE);
    }
    if (g_read_buffer_size == 0) {
        size_t default_read_buffer =
            (g_payload_offset + g_chunk_size) * DEFAULT_READ_BATCH_PACKETS;
        g_read_buffer_size = default_read_buffer > g_transfer_size ?
            default_read_buffer : g_transfer_size;
    }
    if (!g_method_explicit && g_transfer_size > g_chunk_size) {
        g_force_read = 0;
    }
    if (g_payload_offset >= g_read_buffer_size) {
        fprintf(stderr, "payload offset must be smaller than read buffer size\n");
        exit(EXIT_FAILURE);
    }
    if (g_max_frame_size < 1024) {
        fprintf(stderr, "max frame size too small\n");
        exit(EXIT_FAILURE);
    }
}

static int install_signal_handlers(void)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) != 0) {
        return -1;
    }
    if (sigaction(SIGTERM, &sa, NULL) != 0) {
        return -1;
    }

    return 0;
}

static const char *effective_home_dir(void)
{
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user && *sudo_user) {
        struct passwd *pw = getpwnam(sudo_user);
        if (pw && pw->pw_dir && pw->pw_dir[0]) {
            return pw->pw_dir;
        }
    }

    {
        const char *home = getenv("HOME");
        if (home && *home) {
            return home;
        }
    }

    {
        struct passwd *pw = getpwuid(getuid());
        if (pw && pw->pw_dir && pw->pw_dir[0]) {
            return pw->pw_dir;
        }
    }

    return "/tmp";
}

static char *build_output_dir(void)
{
    const char *home = effective_home_dir();
    size_t needed = strlen(home) + strlen("/Desktop/rx_images") + 1;
    char *path = (char *)malloc(needed);
    if (!path) {
        return NULL;
    }
    snprintf(path, needed, "%s/Desktop/rx_images", home);
    return path;
}

static int mkdir_p(const char *path)
{
    char *copy = strdup(path);
    char *p;

    if (!copy) {
        return -1;
    }

    for (p = copy + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(copy, 0775) == -1 && errno != EEXIST) {
                free(copy);
                return -1;
            }
            *p = '/';
        }
    }

    if (mkdir(copy, 0775) == -1 && errno != EEXIST) {
        free(copy);
        return -1;
    }

    free(copy);
    return 0;
}

static void hex_dump_packet(
    unsigned long long packet_index,
    const uint8_t *data,
    size_t len)
{
    size_t i;

    printf("[pkt %llu] %zu bytes\n", packet_index, len);
    for (i = 0; i < len; ++i) {
        if ((i % 16) == 0) {
            printf("%04zx:", i);
        }
        printf(" %02X", data[i]);
        if ((i % 16) == 15 || i + 1 == len) {
            putchar('\n');
        }
    }
}

static void update_stream_stats(
    struct stream_stats *stats,
    const uint8_t *data,
    size_t len)
{
    size_t i;

    for (i = 0; i < len; ++i) {
        uint8_t b = data[i];

        stats->bytes++;
        if (b != 0) {
            stats->nonzero++;
        }
        if (b == JPEG_SOI_0) {
            stats->ff++;
        }

        if (stats->have_prev && stats->prev == JPEG_SOI_0) {
            if (b == JPEG_SOI_1) {
                stats->soi++;
            } else if (b == JPEG_EOI_1) {
                stats->eoi++;
            } else if (b == 0x1B) {
                stats->soi_lsb_first++;
            } else if (b == 0x9B) {
                stats->eoi_lsb_first++;
            }
        }

        stats->prev = b;
        stats->have_prev = 1;
    }
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int ensure_capacity(struct jpeg_state *st, size_t needed)
{
    if (needed <= st->cap) {
        return 0;
    }

    size_t new_cap = st->cap ? st->cap : 4096;
    while (new_cap < needed) {
        new_cap *= 2;
    }

    st->buf = (uint8_t *)realloc(st->buf, new_cap);
    if (!st->buf) {
        return -1;
    }

    st->cap = new_cap;
    return 0;
}

static void reset_capture(struct jpeg_state *st)
{
    st->len = 0;
    st->capturing = 0;
    st->have_tail_prev = 0;
}

static void log_offset_gap(
    struct app_stats *app_stats,
    uint32_t expected,
    uint32_t got,
    size_t jpeg_len)
{
    uint32_t gap_bytes = got - expected;
    uint32_t gap_packets =
        (gap_bytes + APP_PAYLOAD_MAX - 1U) / APP_PAYLOAD_MAX;
    const char *exact = (gap_bytes % APP_PAYLOAD_MAX) == 0U ? "yes" : "no";

    if (gap_packets < GAP_WARN_MIN_PACKETS) {
        app_stats->suppressed_gap_warnings++;
        return;
    }

    if (g_gap_warn_limit == 0U && !g_verbose) {
        app_stats->suppressed_gap_warnings++;
        return;
    }

    if (g_verbose || g_gap_warnings_printed < g_gap_warn_limit) {
        fprintf(stderr,
                "[warn] app offset gap expected=%u, got=%u, missing=%u bytes ~= %u payload packets, exact240=%s, zero-fill gap (image %zu bytes)\n",
                expected, got, gap_bytes, gap_packets, exact, jpeg_len);
        if (!g_verbose) {
            g_gap_warnings_printed++;
            if (g_gap_warnings_printed == g_gap_warn_limit) {
                fprintf(stderr,
                        "[info] further app offset gap warnings suppressed; status still counts gaps\n");
            }
        }
        return;
    }

    app_stats->suppressed_gap_warnings++;
}

static int save_jpeg(const struct jpeg_state *st, const char *output_dir)
{
    time_t now = time(NULL);
    struct tm tm_now;
    char timestamp[64];
    char path[1024];
    FILE *fp;

    if (!localtime_r(&now, &tm_now)) {
        return -1;
    }

    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);
    snprintf(path, sizeof(path), "%s/rx_%s_%04u.jpg",
             output_dir, timestamp, st->image_index);

    fp = fopen(path, "wb");
    if (!fp) {
        perror("fopen");
        return -1;
    }

    if (fwrite(st->buf, 1, st->len, fp) != st->len) {
        perror("fwrite");
        fclose(fp);
        return -1;
    }

    fclose(fp);
    printf("[saved] %s (%zu bytes)\n", path, st->len);
    return 0;
}

static void force_jpeg_markers(struct jpeg_state *st)
{
    if (!st->buf || st->len < 2U) {
        return;
    }

    st->buf[0] = JPEG_SOI_0;
    st->buf[1] = JPEG_SOI_1;
    st->buf[st->len - 2U] = JPEG_EOI_0;
    st->buf[st->len - 1U] = JPEG_EOI_1;
}

static int save_reassembled_image(
    struct jpeg_state *st,
    const char *output_dir,
    unsigned int *saved_count)
{
    if (!st->capturing || st->len == 0U) {
        return 0;
    }

    force_jpeg_markers(st);

    if (save_jpeg(st, output_dir) != 0) {
        return -1;
    }

    (*saved_count)++;
    reset_capture(st);
    return 0;
}

static int start_reassembled_image(struct jpeg_state *st, uint32_t image_size)
{
    if (image_size == 0U || image_size > g_max_frame_size) {
        return -1;
    }

    if (ensure_capacity(st, image_size) != 0) {
        fprintf(stderr, "failed to allocate image reassembly buffer\n");
        return -1;
    }

    memset(st->buf, 0, image_size);
    st->len = image_size;
    st->capturing = 1;
    st->have_prev = 0;
    st->have_tail_prev = 0;
    st->image_index++;

    if (g_verbose) {
        printf("[jpeg] image #%u reassembly started (%u bytes)\n",
               st->image_index, image_size);
    }

    return 0;
}

static int receive_chunk_ioctl(int fd, uint8_t *rx, size_t len)
{
    static uint8_t *tx_dummy = NULL;
    static size_t tx_dummy_len = 0;
    static struct spi_ioc_transfer *transfers = NULL;
    static size_t transfer_count_cap = 0;
    size_t segment_len = g_payload_offset + g_chunk_size;
    size_t transfer_count;
    size_t i;

    if (segment_len == 0 || (len % segment_len) != 0) {
        errno = EINVAL;
        return -1;
    }

    transfer_count = len / segment_len;
    if (transfer_count == 0) {
        errno = EINVAL;
        return -1;
    }

    if (g_tx_dummy && tx_dummy_len < len) {
        uint8_t *new_dummy = (uint8_t *)realloc(tx_dummy, len);
        if (!new_dummy) {
            fprintf(stderr, "failed to allocate tx dummy buffer\n");
            return -1;
        }
        memset(new_dummy + tx_dummy_len, 0, len - tx_dummy_len);
        tx_dummy = new_dummy;
        tx_dummy_len = len;
    }

    if (transfer_count_cap < transfer_count) {
        struct spi_ioc_transfer *new_transfers =
            (struct spi_ioc_transfer *)realloc(transfers,
                                               transfer_count * sizeof(*transfers));
        if (!new_transfers) {
            fprintf(stderr, "failed to allocate spi transfer list\n");
            return -1;
        }
        transfers = new_transfers;
        transfer_count_cap = transfer_count;
    }

    memset(transfers, 0, transfer_count * sizeof(*transfers));
    for (i = 0; i < transfer_count; ++i) {
        uint8_t *rx_seg = rx + (i * segment_len);

        if (g_tx_dummy) {
            transfers[i].tx_buf = (unsigned long)(tx_dummy + (i * segment_len));
        }
        transfers[i].rx_buf = (unsigned long)rx_seg;
        transfers[i].len = (uint32_t)segment_len;
        transfers[i].delay_usecs = g_delay_usecs;
        if (g_speed != 0) {
            transfers[i].speed_hz = g_speed;
        }
        transfers[i].bits_per_word = g_bits;
        transfers[i].cs_change = (i + 1U < transfer_count) ? 1U : 0U;
    }

    return ioctl(fd, SPI_IOC_MESSAGE(transfer_count), transfers);
}

static int receive_chunk_read(int fd, uint8_t *rx, size_t len)
{
    return (int)read(fd, rx, len);
}

static void tune_process_for_rx(void)
{
    if (setpriority(PRIO_PROCESS, 0, -10) != 0 && g_verbose) {
        perror("setpriority");
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0 && g_verbose) {
        perror("mlockall");
    }
}

static void tune_rx_thread_priority(void)
{
    struct sched_param sp;
    int err;

    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = RX_THREAD_PRIORITY;
    err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (err != 0 && g_verbose) {
        fprintf(stderr, "pthread_setschedparam: %s\n", strerror(err));
    }
}

static int rx_queue_init(struct rx_queue *q, unsigned int depth, size_t block_size)
{
    unsigned int i;

    memset(q, 0, sizeof(*q));
    q->storage = (uint8_t *)malloc((size_t)depth * block_size);
    q->free_idx = (unsigned int *)malloc((size_t)depth * sizeof(*q->free_idx));
    q->ready_idx = (unsigned int *)malloc((size_t)depth * sizeof(*q->ready_idx));
    q->lens = (size_t *)calloc(depth, sizeof(*q->lens));
    if (!q->storage || !q->free_idx || !q->ready_idx || !q->lens) {
        free(q->storage);
        free(q->free_idx);
        free(q->ready_idx);
        free(q->lens);
        memset(q, 0, sizeof(*q));
        return -1;
    }

    q->block_size = block_size;
    q->depth = depth;
    q->free_count = depth;
    for (i = 0; i < depth; ++i) {
        q->free_idx[i] = i;
    }

    if (pthread_mutex_init(&q->lock, NULL) != 0 ||
        pthread_cond_init(&q->can_read, NULL) != 0 ||
        pthread_cond_init(&q->can_write, NULL) != 0) {
        free(q->storage);
        free(q->free_idx);
        free(q->ready_idx);
        free(q->lens);
        memset(q, 0, sizeof(*q));
        return -1;
    }

    return 0;
}

static void rx_queue_close(struct rx_queue *q)
{
    pthread_mutex_lock(&q->lock);
    q->closed = 1;
    pthread_cond_broadcast(&q->can_read);
    pthread_cond_broadcast(&q->can_write);
    pthread_mutex_unlock(&q->lock);
}

static void rx_queue_destroy(struct rx_queue *q)
{
    pthread_mutex_destroy(&q->lock);
    pthread_cond_destroy(&q->can_read);
    pthread_cond_destroy(&q->can_write);
    free(q->storage);
    free(q->free_idx);
    free(q->ready_idx);
    free(q->lens);
    memset(q, 0, sizeof(*q));
}

static int rx_queue_acquire_write(struct rx_queue *q, unsigned int *idx, uint8_t **buf)
{
    pthread_mutex_lock(&q->lock);
    while (!q->closed && q->free_count == 0U) {
        pthread_cond_wait(&q->can_write, &q->lock);
    }
    if (q->closed) {
        pthread_mutex_unlock(&q->lock);
        return -1;
    }

    *idx = q->free_idx[q->free_head];
    q->free_head = (q->free_head + 1U) % q->depth;
    q->free_count--;
    *buf = q->storage + ((size_t)(*idx) * q->block_size);
    pthread_mutex_unlock(&q->lock);
    return 0;
}

static void rx_queue_return_slot(struct rx_queue *q, unsigned int idx)
{
    pthread_mutex_lock(&q->lock);
    q->free_idx[q->free_tail] = idx;
    q->free_tail = (q->free_tail + 1U) % q->depth;
    q->free_count++;
    pthread_cond_signal(&q->can_write);
    pthread_mutex_unlock(&q->lock);
}

static void rx_queue_commit_write(struct rx_queue *q, unsigned int idx, size_t len)
{
    pthread_mutex_lock(&q->lock);
    q->lens[idx] = len;
    q->ready_idx[q->ready_tail] = idx;
    q->ready_tail = (q->ready_tail + 1U) % q->depth;
    q->ready_count++;
    pthread_cond_signal(&q->can_read);
    pthread_mutex_unlock(&q->lock);
}

static int rx_queue_pop(struct rx_queue *q, unsigned int *idx, uint8_t **buf, size_t *len)
{
    pthread_mutex_lock(&q->lock);
    while (!q->closed && q->ready_count == 0U) {
        pthread_cond_wait(&q->can_read, &q->lock);
    }
    if (q->ready_count == 0U) {
        pthread_mutex_unlock(&q->lock);
        return 0;
    }

    *idx = q->ready_idx[q->ready_head];
    q->ready_head = (q->ready_head + 1U) % q->depth;
    q->ready_count--;
    *len = q->lens[*idx];
    *buf = q->storage + ((size_t)(*idx) * q->block_size);
    pthread_mutex_unlock(&q->lock);
    return 1;
}

static void print_spidev_size_hint(void)
{
    size_t max_packets =
        SPIDEV_DEFAULT_BUFSIZ_HINT / (g_payload_offset + g_chunk_size);
    fprintf(stderr,
            "[hint] spidev message is larger than the kernel bounce buffer. "
            "Default bufsiz is often %u bytes; try -x %zu or increase spidev.bufsiz.\n",
            SPIDEV_DEFAULT_BUFSIZ_HINT,
            max_packets * (g_payload_offset + g_chunk_size));
}

static void *rx_reader_thread(void *arg)
{
    struct rx_reader_args *args = (struct rx_reader_args *)arg;
    int using_read_method = args->using_read_method;

    tune_rx_thread_priority();

    while (!g_stop) {
        size_t request_len = using_read_method ? g_read_buffer_size : g_transfer_size;
        unsigned int slot = 0;
        uint8_t *rx = NULL;
        int ret;

        if (rx_queue_acquire_write(args->queue, &slot, &rx) != 0) {
            break;
        }

        if (using_read_method) {
            ret = receive_chunk_read(args->fd, rx, request_len);
            if (ret < 0 && errno == EINVAL) {
                fprintf(stderr,
                        "[warn] read() returned EINVAL, retry with SPI_IOC_MESSAGE\n");
                using_read_method = 0;
                request_len = g_transfer_size;
                ret = receive_chunk_ioctl(args->fd, rx, request_len);
                if (ret >= 0) {
                    printf("receive method: SPI_IOC_MESSAGE(%zu)\n",
                           args->spi_transfer_count);
                    fflush(stdout);
                }
            }
        } else {
            ret = receive_chunk_ioctl(args->fd, rx, request_len);
            if (ret < 0 && errno == EINVAL) {
                if (args->spi_transfer_count == 1U) {
                    fprintf(stderr,
                            "[warn] SPI_IOC_MESSAGE(1) returned EINVAL, retry with read() fallback\n");
                    using_read_method = 1;
                    request_len = g_read_buffer_size;
                    ret = receive_chunk_read(args->fd, rx, request_len);
                    if (ret >= 0) {
                        printf("receive method: read()\n");
                        fflush(stdout);
                    }
                } else {
                    fprintf(stderr,
                            "[error] SPI_IOC_MESSAGE(%zu) returned EINVAL; "
                            "do not fallback to one long read(%zu)\n",
                            args->spi_transfer_count,
                            g_read_buffer_size);
                    rx_queue_return_slot(args->queue, slot);
                    break;
                }
            }
        }

        if (ret < 1) {
            int saved_errno = errno;
            rx_queue_return_slot(args->queue, slot);
            if (g_stop || saved_errno == EINTR) {
                break;
            }
            if (saved_errno == EIO || saved_errno == EAGAIN || saved_errno == ETIMEDOUT) {
                usleep(1000);
                continue;
            }
            errno = saved_errno;
            perror("SPI receive");
            if (saved_errno == EMSGSIZE) {
                print_spidev_size_hint();
            }
            break;
        }

        if ((size_t)ret != request_len && g_verbose) {
            static unsigned int short_receive_diag_printed = 0;

            if (short_receive_diag_printed < RX_BLOCK_DIAG_LIMIT) {
                fprintf(stderr, "[diag] short SPI receive: got=%d requested=%zu\n",
                        ret, request_len);
                short_receive_diag_printed++;
                if (short_receive_diag_printed == RX_BLOCK_DIAG_LIMIT) {
                    fprintf(stderr, "[diag] further short receive diagnostics suppressed\n");
                }
            }
        }
        rx_queue_commit_write(args->queue, slot, (size_t)ret);
    }

    rx_queue_close(args->queue);
    return NULL;
}

static int process_app_packet(
    struct jpeg_state *st,
    struct stream_stats *stream_stats,
    struct app_stats *app_stats,
    const uint8_t *data,
    size_t len,
    const char *output_dir,
    unsigned int *saved_count,
    size_t *valid_payload_len)
{
    uint32_t image_size;
    uint32_t offset;
    size_t payload_len;
    size_t max_payload;
    const uint8_t *payload;

    *valid_payload_len = 0;

    if (len < APP_HDR_SIZE) {
        return 0;
    }
    if (data[0] != APP_SYNC) {
        if (app_stats->have_offset) {
            app_stats->no_sync++;
            if (g_verbose) {
                fprintf(stderr, "[warn] missing app packet sync\n");
            }
            return 1;
        }
        return 0;
    }

    if (data[9] != APP_TAIL) {
        app_stats->bad_header++;
        if (g_verbose) {
            fprintf(stderr, "[warn] bad app header tail marker\n");
        }
        return 1;
    }

    image_size = get_le32(&data[1]);
    offset = get_le32(&data[5]);
    max_payload = len - APP_HDR_SIZE;
    if (max_payload > APP_PAYLOAD_MAX) {
        max_payload = APP_PAYLOAD_MAX;
    }

    if (image_size == 0U || image_size > g_max_frame_size) {
        app_stats->bad_len++;
        if (g_verbose) {
            fprintf(stderr, "[warn] bad image size=%u\n", image_size);
        }
        return 1;
    }
    if (offset >= image_size) {
        app_stats->bad_len++;
        if (g_verbose) {
            fprintf(stderr, "[warn] bad app offset=%u, image_size=%u\n",
                    offset, image_size);
        }
        return 1;
    }

    {
        uint32_t image_left = image_size - offset;
        payload_len = image_left < max_payload ? (size_t)image_left : max_payload;
    }

    if (app_stats->have_offset && offset < app_stats->last_offset) {
        app_stats->offset_wraps++;
        if (save_reassembled_image(st, output_dir, saved_count) != 0) {
            return -1;
        }
        st->have_prev = 0;
        st->have_tail_prev = 0;
    } else if (app_stats->have_offset && offset > app_stats->expected_offset) {
        uint32_t gap_bytes = offset - app_stats->expected_offset;
        uint32_t gap_packets =
            (gap_bytes + APP_PAYLOAD_MAX - 1U) / APP_PAYLOAD_MAX;

        app_stats->offset_gaps++;
        app_stats->offset_gap_bytes += gap_bytes;
        app_stats->offset_gap_packets += gap_packets;
        app_stats->last_gap_bytes = gap_bytes;
        app_stats->last_gap_packets = gap_packets;
        if ((gap_bytes % APP_PAYLOAD_MAX) == 0U) {
            app_stats->gap_exact_packets++;
        } else {
            app_stats->gap_partial_packets++;
        }
        log_offset_gap(app_stats, app_stats->expected_offset, offset,
                       st->capturing ? st->len : (size_t)image_size);
    }

    if (!st->capturing || st->len != (size_t)image_size) {
        if (st->capturing) {
            if (save_reassembled_image(st, output_dir, saved_count) != 0) {
                return -1;
            }
        }
        if (start_reassembled_image(st, image_size) != 0) {
            app_stats->bad_len++;
            return 1;
        }
    }

    payload = data + APP_HDR_SIZE;
    memcpy(st->buf + offset, payload, payload_len);
    *valid_payload_len = payload_len;
    update_stream_stats(stream_stats, payload, payload_len);

    app_stats->packets++;
    app_stats->have_offset = 1;
    app_stats->last_offset = offset;
    app_stats->expected_offset = offset + (uint32_t)payload_len;

    if (offset + (uint32_t)payload_len >= image_size) {
        if (save_reassembled_image(st, output_dir, saved_count) != 0) {
            return -1;
        }
    }

    return 1;
}
static int app_stream_reserve(struct app_stream_state *stream, size_t needed)
{
    size_t new_cap;
    uint8_t *new_buf;

    if (needed <= stream->cap) {
        return 0;
    }

    new_cap = stream->cap ? stream->cap : (APP_FRAME_SIZE * 4U);
    while (new_cap < needed) {
        if (new_cap > (SIZE_MAX / 2U)) {
            return -1;
        }
        new_cap *= 2U;
    }

    new_buf = (uint8_t *)realloc(stream->buf, new_cap);
    if (!new_buf) {
        return -1;
    }

    stream->buf = new_buf;
    stream->cap = new_cap;
    return 0;
}

static void app_stream_consume(struct app_stream_state *stream, size_t count)
{
    if (count >= stream->len) {
        stream->len = 0;
        return;
    }

    memmove(stream->buf, stream->buf + count, stream->len - count);
    stream->len -= count;
}

static int app_header_is_plausible(const uint8_t *data, size_t available)
{
    uint32_t image_size;
    uint32_t offset;

    if (available < APP_HDR_SIZE) {
        return 0;
    }
    if (data[0] != APP_SYNC || data[9] != APP_TAIL) {
        return 0;
    }

    image_size = get_le32(&data[1]);
    offset = get_le32(&data[5]);
    if (image_size == 0U || image_size > g_max_frame_size) {
        return 0;
    }
    if (offset >= image_size) {
        return 0;
    }

    return 1;
}

static void diag_rx_block_alignment(
    const uint8_t *data,
    size_t len,
    size_t pending_before,
    struct app_stats *app_stats)
{
    size_t pos;
    size_t expected_phase;
    size_t boundary_count = 0;
    size_t aligned_good = 0;
    size_t aligned_bad = 0;
    size_t phase_count = 0;
    size_t phase_good = 0;
    size_t phase_bad = 0;
    size_t first_bad = SIZE_MAX;
    size_t first_plausible = SIZE_MAX;
    size_t dump_pos;
    size_t dump_len;
    size_t i;

    if (len == 0U) {
        return;
    }

    app_stats->rx_blocks++;
    expected_phase = (APP_FRAME_SIZE - (pending_before % APP_FRAME_SIZE)) %
        APP_FRAME_SIZE;
    app_stats->last_expected_phase = expected_phase;

    for (pos = 0; pos + APP_HDR_SIZE <= len; pos += APP_FRAME_SIZE) {
        boundary_count++;
        if (app_header_is_plausible(data + pos, len - pos)) {
            aligned_good++;
        } else {
            aligned_bad++;
            if (first_bad == SIZE_MAX) {
                first_bad = pos;
            }
        }
    }

    for (pos = expected_phase; pos + APP_HDR_SIZE <= len; pos += APP_FRAME_SIZE) {
        phase_count++;
        if (app_header_is_plausible(data + pos, len - pos)) {
            phase_good++;
        } else {
            phase_bad++;
        }
    }

    for (pos = 0; pos + APP_HDR_SIZE <= len; ++pos) {
        if (app_header_is_plausible(data + pos, len - pos)) {
            first_plausible = pos;
            break;
        }
    }

    app_stats->aligned_headers += aligned_good;
    app_stats->aligned_bad_headers += aligned_bad;
    app_stats->phase_headers += phase_good;
    app_stats->phase_bad_headers += phase_bad;
    if (first_plausible != SIZE_MAX &&
        (first_plausible % APP_FRAME_SIZE) != 0U) {
        app_stats->misaligned_headers++;
    }

    if (!g_verbose || g_rx_block_diag_printed >= RX_BLOCK_DIAG_LIMIT) {
        return;
    }
    if (aligned_bad == 0U &&
        (len % APP_FRAME_SIZE) == 0U &&
        first_plausible == 0U) {
        return;
    }

    fprintf(stderr,
            "[diag] rx block #%llu len=%zu pending_before=%zu expected_phase=%zu phase_good=%zu/%zu phase_bad=%zu boundary0_good=%zu/%zu boundary0_bad=%zu ",
            app_stats->rx_blocks, len, pending_before, expected_phase,
            phase_good, phase_count, phase_bad,
            aligned_good, boundary_count, aligned_bad);
    if (first_bad == SIZE_MAX) {
        fprintf(stderr, "first_bad=none ");
    } else {
        fprintf(stderr, "first_bad=%zu ", first_bad);
    }
    if (first_plausible == SIZE_MAX) {
        fprintf(stderr, "first_plausible=none mod250=none\n");
    } else {
        fprintf(stderr, "first_plausible=%zu mod250=%zu\n",
                first_plausible, first_plausible % APP_FRAME_SIZE);
    }

    if (first_bad != SIZE_MAX) {
        dump_pos = first_bad;
    } else if (first_plausible != SIZE_MAX) {
        dump_pos = first_plausible;
    } else {
        dump_pos = 0U;
    }
    dump_len = len - dump_pos;
    if (dump_len > 16U) {
        dump_len = 16U;
    }

    fprintf(stderr, "[diag] rx bytes @%zu:", dump_pos);
    for (i = 0; i < dump_len; ++i) {
        fprintf(stderr, " %02X", data[dump_pos + i]);
    }
    fputc('\n', stderr);

    g_rx_block_diag_printed++;
    if (g_rx_block_diag_printed == RX_BLOCK_DIAG_LIMIT) {
        fprintf(stderr, "[diag] further rx block diagnostics suppressed\n");
    }
}

static void print_sync_diag(
    const char *reason,
    const uint8_t *data,
    size_t len,
    size_t pos)
{
    size_t i;
    size_t dump_len;

    if (!g_verbose || g_sync_diag_printed >= SYNC_DIAG_LIMIT) {
        return;
    }

    if (pos > len) {
        pos = len;
    }
    dump_len = len - pos;
    if (dump_len > 16U) {
        dump_len = 16U;
    }

    fprintf(stderr, "[diag] %s at stream_pos=%zu pending=%zu bytes:",
            reason, pos, len);
    for (i = 0; i < dump_len; ++i) {
        fprintf(stderr, " %02X", data[pos + i]);
    }
    fputc('\n', stderr);

    g_sync_diag_printed++;
    if (g_sync_diag_printed == SYNC_DIAG_LIMIT) {
        fprintf(stderr, "[diag] further sync diagnostics suppressed; use -v after restart for more\n");
    }
}

static size_t app_stream_find_sync(
    const uint8_t *data,
    size_t len,
    struct app_stats *app_stats)
{
    size_t i;

    if (len < APP_HDR_SIZE) {
        return len;
    }

    for (i = 0; i + APP_HDR_SIZE <= len; ++i) {
        if (data[i] != APP_SYNC) {
            continue;
        }

        app_stats->sync_candidates++;
        if (app_header_is_plausible(data + i, len - i)) {
            return i;
        }

        app_stats->false_sync++;
        print_sync_diag("false AA sync", data, len, i);
    }

    return len;
}

static int process_app_stream(
    struct jpeg_state *st,
    struct stream_stats *stream_stats,
    struct app_stats *app_stats,
    struct app_stream_state *app_stream,
    const uint8_t *data,
    size_t len,
    const char *output_dir,
    unsigned int *saved_count,
    size_t *valid_payload_total)
{
    *valid_payload_total = 0;

    if (len == 0U) {
        return 0;
    }

    if (app_stream_reserve(app_stream, app_stream->len + len) != 0) {
        fprintf(stderr, "failed to grow app stream buffer\n");
        return -1;
    }

    memcpy(app_stream->buf + app_stream->len, data, len);
    app_stream->len += len;

    while (app_stream->len >= APP_HDR_SIZE) {
        size_t valid_payload_len = 0;
        size_t sync_pos = app_stream_find_sync(app_stream->buf,
                                               app_stream->len,
                                               app_stats);

        if (sync_pos == app_stream->len) {
            size_t keep = APP_HDR_SIZE - 1U;
            size_t consume_len = app_stream->len;

            if (consume_len > keep) {
                consume_len -= keep;
            } else {
                break;
            }
            print_sync_diag("no plausible app header", app_stream->buf,
                            app_stream->len, 0U);
            app_stats->no_sync += consume_len;
            app_stream_consume(app_stream, consume_len);
            break;
        }

        if (sync_pos > 0U) {
            print_sync_diag("drop bytes before plausible header", app_stream->buf,
                            app_stream->len, 0U);
            app_stats->no_sync += sync_pos;
            app_stream_consume(app_stream, sync_pos);
        }

        if (app_stream->len < APP_HDR_SIZE) {
            break;
        }

        if (!app_header_is_plausible(app_stream->buf, app_stream->len)) {
            app_stats->bad_header++;
            print_sync_diag("bad app header", app_stream->buf,
                            app_stream->len, 0U);
            app_stream_consume(app_stream, 1U);
            continue;
        }

        if (app_stream->len < APP_FRAME_SIZE) {
            break;
        }

        int status = process_app_packet(st,
                                        stream_stats,
                                        app_stats,
                                        app_stream->buf,
                                        APP_FRAME_SIZE,
                                        output_dir,
                                        saved_count,
                                        &valid_payload_len);
        if (status < 0) {
            return -1;
        }
        if (status == 0) {
            app_stream_consume(app_stream, 1U);
            continue;
        }

        *valid_payload_total += valid_payload_len;
        app_stream_consume(app_stream, APP_FRAME_SIZE);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int ret;
    int using_read_method;
    size_t rx_capacity;
    size_t spi_segment_size;
    size_t spi_transfer_count;
    struct rx_queue rxq;
    pthread_t reader_thread;
    struct rx_reader_args reader_args;
    int reader_started = 0;
    char *output_dir;
    unsigned int saved_count = 0;
    unsigned long long total_rx = 0;
    unsigned long long total_payload = 0;
    unsigned long long packet_count = 0;
    time_t last_status = 0;
    struct jpeg_state st;
    struct stream_stats stream_stats;
    struct app_stats app_stats;
    struct app_stream_state app_stream;

    memset(&st, 0, sizeof(st));
    memset(&stream_stats, 0, sizeof(stream_stats));
    memset(&app_stats, 0, sizeof(app_stats));
    memset(&app_stream, 0, sizeof(app_stream));
    parse_opts(argc, argv);
    using_read_method = g_force_read;
    spi_segment_size = g_payload_offset + g_chunk_size;
    spi_transfer_count = g_transfer_size / spi_segment_size;

    if (install_signal_handlers() != 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    tune_process_for_rx();

    output_dir = build_output_dir();
    if (!output_dir) {
        fprintf(stderr, "failed to allocate output dir path\n");
        return EXIT_FAILURE;
    }

    if (mkdir_p(output_dir) != 0) {
        perror("mkdir_p");
        free(output_dir);
        return EXIT_FAILURE;
    }

    fd = open(g_device, O_RDWR);
    if (fd < 0) {
        perror("open spi device");
        free(output_dir);
        return EXIT_FAILURE;
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE32, &g_mode);
    if (ret == -1) {
        perror("SPI_IOC_WR_MODE32");
        close(fd);
        free(output_dir);
        return EXIT_FAILURE;
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &g_bits);
    if (ret == -1) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        close(fd);
        free(output_dir);
        return EXIT_FAILURE;
    }

    if (g_speed != 0) {
        ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &g_speed);
        if (ret == -1) {
            perror("SPI_IOC_WR_MAX_SPEED_HZ");
            close(fd);
            free(output_dir);
            return EXIT_FAILURE;
        }
    }

    rx_capacity = g_transfer_size;
    if (rx_capacity < g_read_buffer_size) {
        rx_capacity = g_read_buffer_size;
    }

    if (rx_queue_init(&rxq, RX_QUEUE_DEPTH, rx_capacity) != 0) {
        fprintf(stderr, "failed to allocate rx queue\n");
        close(fd);
        free(output_dir);
        return EXIT_FAILURE;
    }

    if (ensure_capacity(&st, 4096) != 0) {
        fprintf(stderr, "failed to allocate jpeg buffer\n");
        rx_queue_destroy(&rxq);
        close(fd);
        free(output_dir);
        return EXIT_FAILURE;
    }

    printf("device: %s\n", g_device);
    printf("mode: 0x%x\n", g_mode);
    printf("bits: %u\n", g_bits);
    if (g_speed != 0) {
        printf("speed: %u Hz\n", g_speed);
    } else {
        printf("speed: unset (slave keeps controller/default setting)\n");
    }
    printf("chunk payload: %zu bytes\n", g_chunk_size);
    printf("spi transfer: %zu bytes total (%zu x %zu-byte SPI transfers)\n",
           g_transfer_size,
           spi_transfer_count,
           spi_segment_size);
    printf("payload offset: %zu bytes\n", g_payload_offset);
    printf("read buffer: %zu bytes\n", g_read_buffer_size);
    printf("rx queue: %u x %zu bytes\n", RX_QUEUE_DEPTH, rx_capacity);
    printf("app protocol: 250-byte frame, AA image_size32 offset32 BB + 240 payload bytes\n");
    printf("output dir: %s\n", output_dir);
    if (using_read_method) {
        printf("receive method: read()\n");
    } else {
        printf("receive method: SPI_IOC_MESSAGE(%zu)\n", spi_transfer_count);
    }
    fflush(stdout);

    reader_args.fd = fd;
    reader_args.using_read_method = using_read_method;
    reader_args.spi_transfer_count = spi_transfer_count;
    reader_args.queue = &rxq;
    if (pthread_create(&reader_thread, NULL, rx_reader_thread, &reader_args) != 0) {
        perror("pthread_create");
        rx_queue_destroy(&rxq);
        free(st.buf);
        close(fd);
        free(output_dir);
        return EXIT_FAILURE;
    }
    reader_started = 1;

    while (!g_stop) {
        unsigned int rx_slot = 0;
        uint8_t *rx = NULL;
        size_t rx_len = 0;
        int pop_status = rx_queue_pop(&rxq, &rx_slot, &rx, &rx_len);

        if (pop_status <= 0) {
            break;
        }

        packet_count++;
        if (g_dump_packets > 0 && packet_count <= g_dump_packets) {
            hex_dump_packet(packet_count, rx, rx_len);
        }

        size_t payload_len = 0;
        const uint8_t *payload = rx;
        if (rx_len > g_payload_offset) {
            payload = rx + g_payload_offset;
            payload_len = rx_len - g_payload_offset;
        }
        total_rx += (unsigned long long)rx_len;
        app_stats.last_rx_len = rx_len;
        if ((rx_len % APP_FRAME_SIZE) != 0U) {
            app_stats.short_reads++;
            if (g_verbose) {
                static unsigned int rx_len_diag_printed = 0;

                if (rx_len_diag_printed < RX_BLOCK_DIAG_LIMIT) {
                    fprintf(stderr, "[diag] rx_len=%zu is not a multiple of %u\n",
                            rx_len, APP_FRAME_SIZE);
                    rx_len_diag_printed++;
                    if (rx_len_diag_printed == RX_BLOCK_DIAG_LIMIT) {
                        fprintf(stderr, "[diag] further rx_len diagnostics suppressed\n");
                    }
                }
            }
        }
        diag_rx_block_alignment(payload, payload_len, app_stream.len, &app_stats);

        size_t valid_payload_len = 0;
        if (process_app_stream(&st,
                               &stream_stats,
                               &app_stats,
                               &app_stream,
                               payload,
                               payload_len,
                               output_dir,
                               &saved_count,
                               &valid_payload_len) != 0) {
            rx_queue_return_slot(&rxq, rx_slot);
            g_stop = 1;
            rx_queue_close(&rxq);
            break;
        }
        rx_queue_return_slot(&rxq, rx_slot);

        total_payload += (unsigned long long)valid_payload_len;

        {
            time_t now = time(NULL);
            if (now != (time_t)-1 &&
                (last_status == 0 || now - last_status >= STATUS_INTERVAL_SEC)) {
                printf("[rx] total=%llu bytes, payload=%llu bytes, app_ok=%llu, no_sync=%llu, bad_header=%llu, bad_len=%llu, false_sync=%llu, candidates=%llu, short_rx=%llu, last_rx=%zu, phase=%zu, phase_ok=%llu, phase_bad=%llu, boundary0_ok=%llu, boundary0_bad=%llu, misalign=%llu, wrap=%llu, gap=%llu, gap_bytes=%llu, gap_pkts~=%llu, gap_exact=%llu, gap_partial=%llu, last_gap=%uB/%upkts, pending=%zu, last_off=%u, soi=%llu, eoi=%llu, saved=%u, current=%zu bytes%s\n",
                       total_rx,
                       total_payload,
                       app_stats.packets,
                       app_stats.no_sync,
                       app_stats.bad_header,
                       app_stats.bad_len,
                       app_stats.false_sync,
                       app_stats.sync_candidates,
                       app_stats.short_reads,
                       app_stats.last_rx_len,
                       app_stats.last_expected_phase,
                       app_stats.phase_headers,
                       app_stats.phase_bad_headers,
                       app_stats.aligned_headers,
                       app_stats.aligned_bad_headers,
                       app_stats.misaligned_headers,
                       app_stats.offset_wraps,
                       app_stats.offset_gaps,
                       app_stats.offset_gap_bytes,
                       app_stats.offset_gap_packets,
                       app_stats.gap_exact_packets,
                       app_stats.gap_partial_packets,
                       app_stats.last_gap_bytes,
                       app_stats.last_gap_packets,
                       app_stream.len,
                       app_stats.last_offset,
                       stream_stats.soi,
                       stream_stats.eoi,
                       saved_count,
                       st.len,
                       st.capturing ? " capturing" : " waiting SOI");
                fflush(stdout);
                last_status = now;
            }
        }

        if (g_max_images > 0 && saved_count >= g_max_images) {
            printf("[info] reached target image count: %u\n", saved_count);
            g_stop = 1;
            rx_queue_close(&rxq);
            break;
        }
    }

    if (reader_started) {
        pthread_join(reader_thread, NULL);
    }

    if (st.capturing) {
        fprintf(stderr, "[info] exit with incomplete reassembled jpeg buffered: %zu bytes\n", st.len);
        (void)save_reassembled_image(&st, output_dir, &saved_count);
    }

    free(st.buf);
    free(app_stream.buf);
    rx_queue_destroy(&rxq);
    close(fd);
    free(output_dir);
    return 0;
}
