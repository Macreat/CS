#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t cap) // manage by apunter or vector
{
    rb->buffer = mem_add;
    rb->capacity = cap;
    ring_buffer_reset(rb);
}

/*
 * @brief Esta función reinicia el buffer circular
 */
void ring_buffer_reset(ring_buffer_t *rb)
{
    rb->head = 0; //La flecha es por que es un apuntador
    rb->tail = 0;
    rb->is_full = 0;
}

/*
 * @brief Esta función calcula la cantidad de datos disponibles en el buffer
 * @retval size: cantidad de datos disponibles
 */
uint8_t ring_buffer_size(ring_buffer_t *rb)
{
    uint8_t size = rb->capacity; // MOVE TO 0?

    if (!rb->is_full)
    {
        if (rb->head >= rb->tail)
        {
            size = rb->head - rb->tail;
        }
        else
        {
            size = rb->capacity + rb->head - rb->tail;
        }
    }

    return size;
}

/*
 * @brief Esta función revisa si el buffer está lleno
 * @retval 1 si está lleno, 0 si no lo está
 */
uint8_t ring_buffer_is_full(ring_buffer_t *rb)
{
    return rb->is_full;
}

/*
 * @brief Esta función revisa si el buffer está vacío
 * @retval 1 si está vacío, 0 si no lo está
 */
uint8_t ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (!rb->is_full && (rb->head == rb->tail)); // ?1 : 0;
}

/*
 * @brief Esta función escribe un dato en el buffer circular
 * @param data: el dato que se va a escribir
 */
void ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    rb->buffer[rb->head] = data;
   // rb->head = rb-> head+ 1 // see logic

    if (rb->is_full)
    {
        rb->tail = (rb->tail + 1) % rb->capacity;
    }

    rb->head = (rb->head + 1) % rb->capacity;

    // Si head alcanza a tail, el buffer está lleno
    if (rb->head == rb->tail)
    {
        rb->is_full = 1;
    }
}

/*
 * @brief Esta función lee un dato del buffer circular
 * @param data: puntero a la variable donde se almacenará el dato leído
 * @retval 1 si hay datos disponibles y se leyó un byte, 0 si el buffer está vacío
 */
uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (ring_buffer_is_empty(rb))
    {
        // Buffer vacío, no hay nada que leer
        return 0;
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->is_full = 0;

    return 1; // Se leyó un byte correctamente
}
