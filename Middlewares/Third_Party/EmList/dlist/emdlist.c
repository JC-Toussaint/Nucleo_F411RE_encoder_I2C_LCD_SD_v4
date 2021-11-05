#include "emdlist.h"

DoubleLinkedList* emdlist_create() {
    DoubleLinkedList* list = (DoubleLinkedList*) malloc(sizeof(DoubleLinkedList));
    emdlist_initialize(list);
    return list;
}

void emdlist_destroy(DoubleLinkedList* list) {
    if(list != NULL) {
        emdlist_deinitialize(list);
        free(list);
    }
}

void emdlist_deinitialize(DoubleLinkedList* list) {
    while(!emdlist_is_empty(list)) {
        emdlist_popfront(list);
    }
}

void emdlist_initialize(DoubleLinkedList* list) {
    list->head = NULL;
    list->tail = NULL;
}

bool emdlist_contains(DoubleLinkedList* list, void* data) {
    for (DoubleLinkedListIterator it = emdlist_iterator(list); it.curr !=NULL; emdlist_iterator_next(&it)){
        DoubleLinkedListElement* candidate = it.curr;
        if (candidate->data == data) {
           return true;
           }
        }
    return false;
}

void emdlist_print(DoubleLinkedList* list, fun_ptr print) {
    printf("head %p tail %p\n", list->head, list->tail);
    for (DoubleLinkedListIterator it = emdlist_iterator(list); it.curr !=NULL; emdlist_iterator_next(&it)){
        print(it.curr);
        }
    printf("----------------------------------------------\n");
}

void emdlist_reverse_print(DoubleLinkedList* list, fun_ptr print) {
    printf("head %p tail %p\n", list->head, list->tail);
    for (DoubleLinkedListIterator it = emdlist_reverse_iterator(list); it.curr !=NULL; emdlist_iterator_prev(&it)){
        print(it.curr);
        }
    printf("----------------------------------------------\n");
}

bool emdlist_pushback(DoubleLinkedList* list, void* data) {
    DoubleLinkedListElement* element = (DoubleLinkedListElement*) malloc(
            sizeof(DoubleLinkedListElement));
    if(element != NULL) {
        element->data = data;
        element->prev = NULL;
        element->next = NULL;
        if(emdlist_is_empty(list)) {
            list->head = element;
            list->tail = element;
        } else {
            DoubleLinkedListIterator iterator = emdlist_iterator(list);
            DoubleLinkedListElement* current = list->tail;
            current->next = element;
            element->prev = current;
            list->tail = element;
        }
        return true;
    }
    return false;
}

bool emdlist_insert(DoubleLinkedList* list, void* data,  cmp_fun_ptr compare) {
	DoubleLinkedListElement* element = (DoubleLinkedListElement*) malloc(sizeof(DoubleLinkedListElement));
	if(element != NULL) {
		element->data = data;
		element->prev = NULL;
		element->next = NULL;
		if(emdlist_is_empty(list)) {
			list->head = element;
			list->tail = element;
            }    
        else{
        DoubleLinkedListIterator it = emdlist_iterator(list);
        DoubleLinkedListElement* curr = it.curr;  
        if (!compare((void*) data, (void*) curr->data)){
           emdlist_pushfront(list, (void*) data);
           return true;
           }

        for (DoubleLinkedListIterator it = emdlist_iterator(list); it.curr !=NULL; emdlist_iterator_next(&it)){
		    DoubleLinkedListElement* curr = it.curr;     
		    DoubleLinkedListElement* next = curr->next; 
            
		    if (!next) {
		       curr->next=element;
		       element->prev=curr;
		       list->tail = element;
		       break;
		       }

            if (compare(data, curr->data) && !compare(data, next->data)){
		       curr->next=element;
		       element->prev=curr;
		       next->prev=element;
		       element->next=next;
               break;   
               }
            }// endfor
         }
     return true;
     }
   return false;
}

bool emdlist_pushfront(DoubleLinkedList* list, void* data) {
    DoubleLinkedListElement* element = (DoubleLinkedListElement*) malloc(sizeof(DoubleLinkedListElement));
    if(element != NULL) {
        element->data = data;
        element->prev = NULL;
        element->next = NULL;
        if(emdlist_is_empty(list)) {
            list->head = element;
            list->tail = element;
        } else {
            DoubleLinkedListIterator it = emdlist_iterator(list);   // = list->head
            DoubleLinkedListElement* current = it.curr; 
            element->next=current;
            current->prev=element;
            list->head = element;
        }
        return true;
    }
    return false;
}

bool emdlist_remove(DoubleLinkedList* list, void* data) {
    DoubleLinkedListElement* prev = NULL;

    for (DoubleLinkedListIterator it = emdlist_iterator(list); it.curr !=NULL; emdlist_iterator_next(&it)){
        DoubleLinkedListElement* curr = it.curr;
        if(curr->data == data) {
            DoubleLinkedListElement* next = curr->next;
            if(prev == NULL) {
                list->head = next;
            } else {
                prev->next = next;
                if (next != NULL) next->prev=prev;
                
            }
            free(curr);
            return true;
        }
        prev = curr;
        }
    return false;
}

void* emdlist_popfront(DoubleLinkedList* list) {
    DoubleLinkedListElement* element = list->head;

    if(element != NULL) {
        list->head = element->next;
        if (list->head == NULL)
           list->tail=NULL;
        else list->head->prev = NULL;
    }

    void* data = element->data;
    free(element);
    return data;
}

void* emdlist_popback(DoubleLinkedList* list) {
    DoubleLinkedListElement* element = list->tail;
    if(element != NULL) {
        list->tail = element->prev;
        if (list->tail == NULL)
           list->head=NULL;
        else list->tail->next=NULL;
    }
    void* data = element->data;
    free(element);
    return data;
}

int emdlist_size(DoubleLinkedList* list) {
    int size = 0;
    DoubleLinkedListIterator iterator = emdlist_iterator(list);
    DoubleLinkedListElement* element = NULL;
    while((element = emdlist_iterator_next(&iterator)) != NULL) {
        ++size;
    }
    return size;
}

bool emdlist_is_empty(DoubleLinkedList* list) {
    return list != NULL && list->head == NULL && list->tail == NULL;
}

DoubleLinkedListIterator emdlist_iterator(DoubleLinkedList* list) {
    DoubleLinkedListIterator iterator;
    iterator.curr = (list != NULL ? list->head : NULL); 
    iterator.prev = NULL;
    iterator.next = (iterator.curr != NULL ? iterator.curr->next : NULL);
    return iterator;
}

DoubleLinkedListIterator emdlist_reverse_iterator(DoubleLinkedList* list) {
    DoubleLinkedListIterator iterator;
    iterator.curr = (list != NULL ? list->tail : NULL); 
    iterator.prev = (iterator.curr != NULL ? iterator.curr->prev : NULL);
    iterator.next = NULL;
    return iterator;
}

DoubleLinkedListElement* emdlist_iterator_next(DoubleLinkedListIterator* iterator) {
    DoubleLinkedListElement* prev = NULL;
    DoubleLinkedListElement* next = NULL;
    if(iterator != NULL) {
        next = iterator->next;
        iterator->curr = next;
        iterator->prev = (next != NULL ? next->prev : NULL);
        iterator->next = (next != NULL ? next->next : NULL);
    }
    return next;
}

DoubleLinkedListElement* emdlist_iterator_prev(DoubleLinkedListIterator* iterator) {
    DoubleLinkedListElement* prev = NULL;
    DoubleLinkedListElement* next = NULL;
    if(iterator != NULL) {
        prev = iterator->prev;
        iterator->curr = prev;
        iterator->prev = (prev != NULL ? prev->prev : NULL);
        iterator->next = (prev != NULL ? prev->next : NULL);
    }
    return prev;
}
