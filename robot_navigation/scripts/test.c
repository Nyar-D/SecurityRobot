#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

typedef struct data {
    int value;
    int* ptr;
    char* pch;
    struct data* dp;
} data;

int main()
{
    // 段错误
    // char* p1 = (char *)0x22;
    // printf("%c\n", *p1);

    // 正常
    // free(p2);

    // char var = 'a';
    // char *p2 = &var;
    // printf("%p\n", p2);

    // char * p3 = (char *)0x7ffd2fbaf537;
    // printf("%p\n", p3);
    // *p3 = 'b';
    // printf("%c\n", *p3);

    data* td = (data*)malloc(sizeof(data));
    td->value = 300;
    int num = 200;
    char ch = 'a';
    data* p = (data*)malloc(sizeof(data));
    p->value = 100;
    p->ptr = &num;
    p->dp = td;
    p->pch = &ch;
    
    printf("%d\n", p->value);
    printf("%d\n", *p->ptr);
    printf("%c\n", *p->pch);

    printf("%p\n", &p->value);
    printf("%p\n", p->ptr);
    printf("%p\n", p->dp);
    printf("%p\n", p->pch);
    free(p);
    // p = NULL;
    // while (1) {
    printf("%d\n", p->value);
    printf("%d\n", *p->ptr);
    printf("%c\n", *p->pch);

    printf("%p\n", &p->value);
    printf("%p\n", p->ptr);
    printf("%p\n", p->dp);
    printf("%p\n", p->pch);
    // }

    // unsigned long int i = (long int)p;
    // data *p1 = (data *)i;
    // printf("%ld\n", i);
    // printf("%d\n", p1->value);

    // char str[3] = {'1', '1', '1'};
    // char *p = str;
    // int *intp = (int *)p;

    // *(p + 5) = 'g';
    // printf("%p\n", p);

    // printf("%c\n", *(p+5));
    // unsigned long int i = (unsigned long int)(p+5);
    // printf("%p\n", p);
    // printf("%lx\n", i);

    // char *p4 = (char *)i;
    // *p4 = 's';
    // printf("%c\n", *p4);

    // 正常
    // char *p3 = (char *)p2;
    // *p3 = 'b';
    // printf("%c\n", *p3);

    // // 段错误
    // char *p4 = (char *)0x56178a4612a0;
    // *p4 = 'b';
    // printf("%c\n", *p4);

    return 0;
}