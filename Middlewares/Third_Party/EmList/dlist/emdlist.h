#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef _emdlist_H_
#define _emdlist_H_

#ifdef __cplusplus
extern "C" {
#endif

struct DoubleLinkedListElement {
   void* value;
   struct DoubleLinkedListElement* prev;
   struct DoubleLinkedListElement* next;
};
typedef struct DoubleLinkedListElement DoubleLinkedListElement;

/* Public: A singly linked list of void pointers.
 */
struct DoubleLinkedList {
   DoubleLinkedListElement* head;
   DoubleLinkedListElement* tail;
};
typedef struct DoubleLinkedList DoubleLinkedList;

struct DoubleLinkedListIterator {
   DoubleLinkedListElement* prev;
   DoubleLinkedListElement* next;
};
typedef struct DoubleLinkedListIterator DoubleLinkedListIterator;

typedef void (*fun_ptr)(DoubleLinkedListElement* candidate);

/* Public: Allocate and initialize a new, empty DoubleLinkedList.
 *
 * This allocates memory for the list, so make sure to call
 * emdlist_destroy(DoubleLinkedList*) when you're done.
 *
 * Returns a pointer to a new DoubleLinkedList.
 */
DoubleLinkedList* emdlist_create();

/* Public: De-allocate memory for a list allocated on the heap.
 *
 * Be aware that if the list is not empty, this will leak memory - you must
 * remove and de-allocate the values in the list yourself if they were not on
 * the stack.
 */
void emdlist_destroy(DoubleLinkedList* list);

/* Public: Initialize a DoubleLinkedList that has already been allocated, either
 * in static or dynamic memory.
 *
 * If the list has already been initialized and has elements, this will leak
 * memory.
 *
 * list - the list to initialize.
 */
void emdlist_initialize(DoubleLinkedList* list);

/* Public: Free any memory associated with the list.
 *
 * Be aware that if the list is not empty, this will leak memory - you must
 * remove and de-allocate the values in the list yourself if they were not on
 * the stack.
 *
 * list - the list to de-initialize, created on the stack or heap.
 */
void emdlist_deinitialize(DoubleLinkedList* list);

/* Public: Check if the list contains a value.
 *
 * This function is O(n).
 *
 * Returns true if the value is in the list.
 */
bool emdlist_contains(DoubleLinkedList* list, void* value);

/* Public: append a value into the list
 *
 * This function is O(1).
 *
 * Returns true if the value was inserted successfully. Returns false if memory
 * could not be allocated for the new element.
 */
bool emdlist_pushback (DoubleLinkedList* list, void* value);
bool emdlist_insert(DoubleLinkedList* list, void* value);

/* Public: prepend a value into the list.
 *
 * This function is O(1).
 *
 * Returns true if the value was inserted successfully. Returns false if memory
 * could not be allocated for the new element.
 */
bool emdlist_pushfront(DoubleLinkedList* list, void* value);

/* Public: Remove a value from the list, if it is in the list.
 *
 * This does not free any memory associated with the value, so remember to free
 * it yourself after removing from the list if necessary - you will leak memory
 * otherwise.
 *
 * This function is O(n).
 *
 * Returns true if the value was found in the list and succesfully removed.
 * Returns false if the value was not in the list.
 */
bool emdlist_remove(DoubleLinkedList* list, void* value);

/* Public: Remove and return the first value in the list.
 *
 * This does not free any memory associated with the value, so remember to free
 * it yourself after removing from the list if necessary - you will leak memory
 * otherwise.
 *
 * This function is O(1).
 *
 * Returns the removed value if the list was not empty, otherwise NULL.
 */
void* emdlist_popfront(DoubleLinkedList* list);
void* emdlist_popback (DoubleLinkedList* list);

/* Public: Determine the total size of the list.
 *
 * This function is O(n).
 *
 * Returns the number of elements in the list.
 */
int emdlist_size(DoubleLinkedList* list);

/* Public: Print the list content
 *
 * This function is O(n).
 *
 * Returns true if the value is in the list.
 */
void emdlist_print(DoubleLinkedList* list, fun_ptr print);

/* Public: Print the list content
 *
 * This function is O(n).
 *
 * Returns true if the value is in the list.
 */
void emdlist_reverse_print(DoubleLinkedList* list, fun_ptr print);

/* Public: Check if the list is empty.
 *
 * This function is O(1).
 *
 * Returns true if the list is empty.
 */
bool emdlist_is_empty(DoubleLinkedList* list);

DoubleLinkedListIterator emdlist_iterator(DoubleLinkedList* list);

DoubleLinkedListIterator emdlist_reverse_iterator(DoubleLinkedList* list);

DoubleLinkedListElement*  emdlist_iterator_next(DoubleLinkedListIterator* iterator);

DoubleLinkedListElement*  emdlist_iterator_prev(DoubleLinkedListIterator* iterator);

#ifdef __cplusplus
}
#endif

#endif // _emdlist_H_
