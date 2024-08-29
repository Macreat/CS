
#include "ring_buffer.h"

#define capacity (8)
uint8_t ring_buffer[capacity];
uint8_t head_ptr;
uint8_t tail_ptr;
uint8_t is_full;


/*
 * @brief This functions reset the buffer
 */
void ring_buffer_reset(void)
{
    head_ptr = 0;
    tail_ptr = 0;
    is_full = 0;
}

/*
 * @brief This function calculates the data available in the buffer
 *
 * @retval size: amount of data available
 */
uint8_t ring_buffer_size(void)
{
    uint8_t size = capacity;

    if (!is_full) // is_full == 0, to  != isnt does the negation
    {
        if (head_ptr >= tail_ptr)
        {
            size = head_ptr - tail_ptr;
        }
        else
        {
            size = capacity + (head_ptr - tail_ptr);
        }
    }

    return size;
}

/*
 * @brief This function indicates whether the circular buffer is full
 *
 * @retval 1: full buffer , 0: buffer with available memory
 */
uint8_t ring_buffer_is_full(void)
{
    return is_full;
}

/*
 * @brief  function that indicates whether the circular buffer is empty
 * @retval 1: empty buffer, 0: buffer with available data
 */
uint8_t ring_buffer_is_empty(void)
{
    return (!is_full && (head_ptr == tail_ptr)); // ? 1: 0 to be specify
}


/**
 * @brief This function write a data to the circular buffer
 *
 * @param where data is the data to type
 *
 * @retval null
 */
void ring_buffer_write(uint8_t data)
{
	ring_buffer[head_ptr] = data;
	head_ptr = head_ptr + 1;

	if (head_ptr >= capacity) { // if the head reaches the end of the memory
	  head_ptr = 0;
	}

	if (is_full != 0) { // if old data is lost
	  tail_ptr = tail_ptr + 1;
	}

	if (tail_ptr >= capacity) { // if the tail reaches the end of memory
	  tail_ptr = 0;
	}

	if (head_ptr == tail_ptr) { // if the head reaches the tail
	  is_full = 1;
	}
}

/**
 * @brief Function that reads data from the cricular buffer Esta funcion lee un dato del buffer circular
 *
 * @param data: the adress where the data will be written la direccion de donde se va a escribir el dato
 *
 * @retval 1: there is some data available, 0: no data available
 */
uint8_t ring_buffer_read(uint8_t *data) // 0x20
{
	if ((is_full != 0) || (head_ptr != tail_ptr)) { // data available
		*data = ring_buffer[tail_ptr]; // add: 0x20, val: buffer
		tail_ptr = tail_ptr + 1;
		if (tail_ptr >= capacity) {
			tail_ptr = 0;
		}
		is_full = 0;

		return 1; // full  buffer
	}
	return 0; //empty  buffer
}

