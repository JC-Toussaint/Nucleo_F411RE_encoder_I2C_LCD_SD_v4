This library is licensed under the BSD license

## Contributors
* Chris Peplin, cpeplin@ford.com
* JC Toussaint, jean-christophe.toussaint@grenoble-inp.fr

## example
#include "emlist.h"

typedef 
struct element{
   int number;
   double value;
} element;

void print(LinkedListElement* candidate){
   element *e=(element*) candidate->value;
   printf("%p\n", e);
   if (e) printf("'--> %d %f\n", e->number, e->value);
}

int main(void) {
    LinkedList* list=emlist_create();

    element *e1= (element*) malloc(sizeof(element));
    e1->number= 1;
    e1->value = 1.0;
    emlist_pushback(list, (void*) e1);

    element *e2= (element*) malloc(sizeof(element));
    e2->number= 2;
    e2->value = 2.0;
    emlist_pushback(list, (void*) e2);

    emlist_print(list, print);

    element *e3= (element*) malloc(sizeof(element));
    e3->number= 3;
    e3->value = 3.0;
    emlist_pushfront(list, (void*) e3);

    element *e4= (element*) malloc(sizeof(element));
    e4->number= 4;
    e4->value = 4.0;
    emlist_pushback(list, (void*) e4);

    element *e5=NULL;
    emlist_pushback(list, (void*) e5);

    element *e6=NULL;
    emlist_pushback(list, (void*) e6);

    emlist_print(list, print);
    return 0;
}
