/*
 * ringbuf.c - C ring buffer (FIFO) implementation.
 *
 */

#include "ringbuf.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <unistd.h>
//#include <sys/param.h>
//#include <assert.h>
#include "debuglog.h"
#include "memory.h"
#define assert( x ) if( ( x ) == 0 ) { DLOG_Error("os crash, file:%s, line:%d", __FILE__, __LINE__); for( ;; ); }	


//typedef unsigned char uint8_t;

#define MIN(a,b) (((a) >= (b)) ? (b) : (a))
/*
 * The code is written for clarity, not cleverness or performance, and
 * contains many assert()s to enforce invariant assumptions and catch
 * bugs. Feel free to optimize the code and to remove asserts for use
 * in your own projects, once you're comfortable that it functions as
 * intended.
 */

struct ringbuf_t
{
    uint8_t *buf;
    uint8_t *head, *tail;
    size_t size;
};

#define RB_NUM 2
#define RB_LEN (1024)
struct ringbuf_t rb[RB_NUM];
uint8_t rb_buf[RB_NUM][RB_LEN+1];

ringbuf_t
ringbuf_new(size_t capacity)
{
    if(capacity < RB_NUM)
    {
        ringbuf_t tmp_rb = &(rb[capacity]);
        if (tmp_rb) {
        
            /* One byte is used for detecting the full condition. */
            tmp_rb->size = RB_LEN + 1;
            tmp_rb->buf = rb_buf[capacity];
            if (tmp_rb->buf)
                ringbuf_reset(tmp_rb);
            else {
                return 0;
            }
        }
        return tmp_rb;
    }
    
    ringbuf_t rb = malloc_safe(sizeof(struct ringbuf_t));
    if (rb) {
        /* One byte is used for detecting the full condition. */
        rb->size = capacity + 1;
        rb->buf = malloc_safe(rb->size);
        if (rb->buf)
            ringbuf_reset(rb);
        else {
            free_safe(rb);
            return 0;
        }
    }
    return rb;

}

size_t
ringbuf_buffer_size(const struct ringbuf_t *rb)
{
    return rb->size;
}

void
ringbuf_reset(ringbuf_t rb)
{
    rb->head = rb->tail = rb->buf;
}

void
ringbuf_free(ringbuf_t *rb)
{
    assert(rb && *rb);
    *rb = 0;
}

size_t
ringbuf_capacity(const struct ringbuf_t *rb)
{
    return ringbuf_buffer_size(rb) - 1;
}

/*
 * Return a pointer to one-past-the-end of the ring buffer's
 * contiguous buffer. You shouldn't normally need to use this function
 * unless you're writing a new ringbuf_* function.
 */
static const uint8_t *
ringbuf_end(const struct ringbuf_t *rb)
{
    return rb->buf + ringbuf_buffer_size(rb);
}

size_t
ringbuf_bytes_free(const struct ringbuf_t *rb)
{
    if (rb->head >= rb->tail)
        return (ringbuf_capacity(rb) - (rb->head - rb->tail));
    else
        return (rb->tail - rb->head - 1);
}

size_t
ringbuf_bytes_used(const struct ringbuf_t *rb)
{
    return ringbuf_capacity(rb) - ringbuf_bytes_free(rb);
}

int
ringbuf_is_full(const struct ringbuf_t *rb)
{
    return ringbuf_bytes_free(rb) == 0;
}

int
ringbuf_is_empty(const struct ringbuf_t *rb)
{
    return ringbuf_bytes_free(rb) == ringbuf_capacity(rb);
}

const void *
ringbuf_tail(const struct ringbuf_t *rb)
{
    return rb->tail;
}

const void *
ringbuf_head(const struct ringbuf_t *rb)
{
    return rb->head;
}

/*
 * Given a ring buffer rb and a pointer to a location within its
 * contiguous buffer, return the a pointer to the next logical
 * location in the ring buffer.
 */
static uint8_t *
ringbuf_nextp(ringbuf_t rb, const uint8_t *p)
{
    /*
     * The assert guarantees the expression (++p - rb->buf) is
     * non-negative; therefore, the modulus operation is safe and
     * portable.
     */
    //assert((p >= rb->buf) && (p < ringbuf_end(rb)));
    return (rb->buf + ((++p - rb->buf) % ringbuf_buffer_size(rb)));
}

size_t
ringbuf_findchr(const struct ringbuf_t *rb, int c, size_t offset)
{
    const uint8_t *bufend = ringbuf_end(rb);
    size_t bytes_used = ringbuf_bytes_used(rb);
    if (offset >= bytes_used)
        return bytes_used;

    const uint8_t *start = rb->buf +
        (((rb->tail - rb->buf) + offset) % ringbuf_buffer_size(rb));
    assert(bufend > start);
    size_t n = MIN(bufend - start, bytes_used - offset);
    const uint8_t *found = memchr(start, c, n);
    if (found)
        return offset + (found - start);
    else
        return ringbuf_findchr(rb, c, offset + n);
}

size_t
ringbuf_memset(ringbuf_t dst, int c, size_t len)
{
    const uint8_t *bufend = ringbuf_end(dst);
    size_t nwritten = 0;
    size_t count = MIN(len, ringbuf_buffer_size(dst));
    int overflow = count > ringbuf_bytes_free(dst);

    while (nwritten != count) {

        /* don't copy beyond the end of the buffer */
        //assert(bufend > dst->head);
        size_t n = MIN(bufend - dst->head, count - nwritten);
        memset(dst->head, c, n);
        dst->head += n;
        nwritten += n;

        /* wrap? */
        if (dst->head == bufend)
            dst->head = dst->buf;
    }

    if (overflow) {
        dst->tail = ringbuf_nextp(dst, dst->head);
        assert(ringbuf_is_full(dst));
    }

    return nwritten;
}

void *
ringbuf_memcpy_into(ringbuf_t dst, const void *src, size_t count)
{
    const uint8_t *u8src = src;
    const uint8_t *bufend = ringbuf_end(dst);
    int overflow = count > ringbuf_bytes_free(dst);
    size_t nread = 0;

    while (nread != count) {
        /* don't copy beyond the end of the buffer */
        assert(bufend > dst->head);
        size_t n = MIN(bufend - dst->head, count - nread);
        memcpy(dst->head, u8src + nread, n);
        dst->head += n;
        nread += n;

        /* wrap? */
        if (dst->head >= bufend)
            dst->head = dst->buf;
    }

    if (overflow) {
        //dst->tail = ringbuf_nextp(dst, dst->head);
        //assert(ringbuf_is_full(dst));
        DLOG_Warning("overflow");
    }

    return dst->head;
}

void *
ringbuf_memcpy_from(void *dst, ringbuf_t src, size_t count)
{
    size_t bytes_used = ringbuf_bytes_used(src);
    if (count > bytes_used)
        return 0;

    uint8_t *u8dst = dst;
    const uint8_t *bufend = ringbuf_end(src);
    size_t nwritten = 0;
    while (nwritten != count) {
        assert(bufend > src->tail);
        size_t n = MIN(bufend - src->tail, count - nwritten);
        memcpy(u8dst + nwritten, src->tail, n);
        src->tail += n;
        nwritten += n;

        /* wrap ? */
        if (src->tail >= bufend)
            src->tail = src->buf;
    }

    //assert(count + ringbuf_bytes_used(src) == bytes_used);
    return src->tail;
}

void *
ringbuf_copy(ringbuf_t dst, ringbuf_t src, size_t count)
{
    size_t src_bytes_used = ringbuf_bytes_used(src);
    if (count > src_bytes_used)
        return 0;
    int overflow = count > ringbuf_bytes_free(dst);

    const uint8_t *src_bufend = ringbuf_end(src);
    const uint8_t *dst_bufend = ringbuf_end(dst);
    size_t ncopied = 0;
    while (ncopied != count) {
        assert(src_bufend > src->tail);
        size_t nsrc = MIN(src_bufend - src->tail, count - ncopied);
        assert(dst_bufend > dst->head);
        size_t n = MIN(dst_bufend - dst->head, nsrc);
        memcpy(dst->head, src->tail, n);
        src->tail += n;
        dst->head += n;
        ncopied += n;

        /* wrap ? */
        if (src->tail == src_bufend)
            src->tail = src->buf;
        if (dst->head == dst_bufend)
            dst->head = dst->buf;
    }

    assert(count + ringbuf_bytes_used(src) == src_bytes_used);
    
    if (overflow) {
        dst->tail = ringbuf_nextp(dst, dst->head);
        assert(ringbuf_is_full(dst));
    }

    return dst->head;
}
