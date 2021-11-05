This library is licensed under the BSD license

## Contributors
* Chris Peplin, cpeplin@ford.com
* JC Toussaint, jean-christophe.toussaint@grenoble-inp.fr

## example
#include "emdlist.h"

typedef 
struct element{
   int number;
   double value;
} element_t;

void print(DoubleLinkedListElement* candidate){
   element_t *e=(element_t*) candidate->data;
   printf("%p prev %p next %p", candidate, candidate->prev, candidate->next);
   if (e) printf(" element %p --> %d %f", e, e->number, e->value);
   else printf(" nil");
   printf("\n");
}

bool greater(element_t* e1, element_t* e2){
     if (e1->value > e2->value)
        return true;
     else
        return false;
}

int main(void) {
    DoubleLinkedList* list=emdlist_create();

    element_t *e1= (element_t*) malloc(sizeof(element_t));
    e1->number= 1;
    e1->value = 1.0;
    emdlist_pushback(list, (void*) e1);

    element_t *e2= (element_t*) malloc(sizeof(element_t));
    e2->number= 2;
    e2->value = 2.0;
    emdlist_pushback(list, (void*) e2);

    printf("print\n");
    emdlist_print(list, print);

    printf("print 1 2\n");
    emdlist_print(list, print);

    printf("reverse print 2 1\n");
    emdlist_reverse_print(list, print);

    element_t *e3= (element_t*) malloc(sizeof(element_t));
    e3->number= 3;
    e3->value = 3.0;
    emdlist_pushfront(list, (void*) e3);

    element_t *e4= (element_t*) malloc(sizeof(element_t));
    e4->number= 4;
    e4->value = 4.0;
    emdlist_pushback(list, (void*) e4);

    printf("print 3 1 2 4\n");
    emdlist_print(list, print);

    element_t *e5=NULL;
    emdlist_pushback(list, (void*) e5);

    element_t *e6=NULL;
    emdlist_pushback(list, (void*) e6);

    printf("print 3 1 2 4 nil nil\n");
    emdlist_print(list, print);

    emdlist_popback(list);
    emdlist_popfront(list);
    emdlist_popback(list);

    printf("print 1 2 4\n");
    emdlist_print(list, print);

    emdlist_popfront(list);
    printf("print 2 4\n");
    emdlist_print(list, print);

    element_t *e7= (element_t*) malloc(sizeof(element_t));
    e7->number= 7;
    e7->value = 7.0;
    emdlist_pushback(list, (void*) e7);

    printf("print 2 4 7\n");
    emdlist_print(list, print);

    emdlist_remove(list, e4);
    printf("after removing e4 -> print 2 7\n");
    emdlist_print(list, print);

    emdlist_pushback(list, (void*) e1);
    emdlist_pushback(list, (void*) e1);
    printf("print 2 7 1 1\n");
    emdlist_print(list, print);

    printf("print 2 7 1\n");
    emdlist_remove(list, e1);
    emdlist_print(list, print);

    emdlist_pushfront(list, (void*) e3);
    emdlist_pushback(list, (void*) e3);

    printf("print 3 2 7 1 3\n");
    emdlist_print(list, print);

    emdlist_remove(list, e1);
    printf("after removing 1 -> print 3 2 7 3\n");
    emdlist_print(list, print);

    printf("after removing 1 again -> print 3 2 7 3\n");
    emdlist_remove(list, e1);
    emdlist_print(list, print);
    emdlist_popback(list);
    emdlist_popfront(list);
    emdlist_popback(list);
    emdlist_print(list, print);

    emdlist_deinitialize(list);


    emdlist_pushback(list, (void*) e1);
    emdlist_pushback(list, (void*) e3);
    emdlist_pushback(list, (void*) e7);

    element_t *e= (element_t*) malloc(sizeof(element_t));
    e->number= 4;
    e->value = 2.0;

    emdlist_print(list, print);

{
	DoubleLinkedListElement* element = (DoubleLinkedListElement*) malloc(sizeof(DoubleLinkedListElement));
	if(element != NULL) {
		element->data = (void*)e;
		element->prev = NULL;
		element->next = NULL;
		if(emdlist_is_empty(list)) {
			list->head = element;
			list->tail = element;
            }    
        else{
        DoubleLinkedListIterator it = emdlist_iterator(list);
        DoubleLinkedListElement* curr = it.curr;  
        element_t *e_curr=curr->data;
        if (!greater((void*) e, (void*) e_curr)){
           emdlist_pushfront(list, (void*) e);
           goto fin;
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

            element_t *e_curr=curr->data;
            element_t *e_next=next->data;
            fprintf(stderr, "e->value %lf e_curr->value %lf = %d\n", e->value, e_curr->value, greater((void*) e, (void*) e_curr));
            fprintf(stderr, "e->value %lf e_next->value %lf = %d\n", e->value, e_next->value, greater((void*) e, (void*) e_next));
            if (greater((void*) e, (void*) e_curr) && !greater((void*) e, (void*) e_next)){
               fprintf(stderr, "ici\n");
		       curr->next=element;
		       element->prev=curr;
		       next->prev=element;
		       element->next=next;
               break;   
               }
            }// endfor
         }
     }
fin: printf("\n");
}

    emdlist_print(list, print);

    emdlist_deinitialize(list);

    emdlist_insert(list, (void*) e3, (cmp_fun_ptr) greater);
    emdlist_insert(list, (void*) e1, (cmp_fun_ptr) greater);

    element_t *e8= (element_t*) malloc(sizeof(element_t));
    e8->number= 8;
    e8->value = 8.0;
    emdlist_insert(list, (void*) e8, (cmp_fun_ptr) greater);
    emdlist_print(list, print);

    emdlist_insert(list, (void*) e4, (cmp_fun_ptr) greater);
    emdlist_print(list, print);

    emdlist_insert(list, (void*) e7, (cmp_fun_ptr) greater);
    emdlist_print(list, print);

    return 0;
}
