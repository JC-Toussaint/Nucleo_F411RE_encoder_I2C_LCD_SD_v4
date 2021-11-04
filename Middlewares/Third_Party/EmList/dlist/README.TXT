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
} element;

void print(DoubleLinkedListElement* candidate){
   element *e=(element*) candidate->value;
   printf("%p prev %p next %p", candidate, candidate->prev, candidate->next);
   if (e) printf(" element %p --> %d %f", e, e->number, e->value);
   printf("\n");
}

int main(void) {
    DoubleLinkedList* list=emdlist_create();

    element *e1= (element*) malloc(sizeof(element));
    e1->number= 1;
    e1->value = 1.0;
    emdlist_pushback(list, (void*) e1);

    element *e2= (element*) malloc(sizeof(element));
    e2->number= 2;
    e2->value = 2.0;
    emdlist_pushback(list, (void*) e2);

    emdlist_print(list, print);

    element *e3= (element*) malloc(sizeof(element));
    e3->number= 3;
    e3->value = 3.0;
    emdlist_pushfront(list, (void*) e3);

    element *e4= (element*) malloc(sizeof(element));
    e4->number= 4;
    e4->value = 4.0;
    emdlist_pushback(list, (void*) e4);

    element *e5=NULL;
    emdlist_pushback(list, (void*) e5);

    element *e6=NULL;
    emdlist_pushback(list, (void*) e6);

    emdlist_print(list, print);

    emdlist_popback(list);
    emdlist_popfront(list);
    emdlist_popback(list);
    emdlist_print(list, print);

    emdlist_popfront(list);
    emdlist_print(list, print);

    element *e7= (element*) malloc(sizeof(element));
    e7->number= 7;
    e7->value = 7.0;
    emdlist_pushback(list, (void*) e7);
    emdlist_print(list, print);

    emdlist_remove(list, e4);
    emdlist_print(list, print);

    emdlist_pushback(list, (void*) e1);
    emdlist_pushback(list, (void*) e1);
    emdlist_print(list, print);

    emdlist_remove(list, e1);
    emdlist_print(list, print);

    emdlist_pushfront(list, (void*) e3);
    emdlist_pushback(list, (void*) e3);

    emdlist_print(list, print);

    emdlist_remove(list, e1);
    emdlist_print(list, print);

    emdlist_remove(list, e1);
    emdlist_print(list, print);
    return 0;
}
