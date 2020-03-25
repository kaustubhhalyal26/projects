#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void header()
{
    printf("\n       Flight              Price     Seating Capacity        Available Seats\n");
    printf("\n-----------------------------------------------------------------------------------\n");
}
void option1()
{
    printf("\nFlightNo          Flight                Price   Seating Capacity    Available Seats\n");
    printf("\n-----------------------------------------------------------------------------------\n");
    printf("     1        India To United states      1,500        100                 16\n");
    printf("     2        India To China              1,000        75                  0\n");
    printf("     3        India To Singapore          3,000        100                 24\n");
}
void option2()
{
    printf("\nnFlightNo         Flight                 Price   Seating Capacity    Available Seats\n");
    printf("\n-----------------------------------------------------------------------------------\n");
    printf("     1         India To United states       1,700       100                 18\n");
    printf("     2         India To China               2,000       75                  24\n");
    printf("     3         India To Singapore           4,000       100                 65\n");
}
void option3()
{
    printf("\n.nFlightNo        Flight                    Price   Seating Capacity    Available Seats\n");
    printf("\n-----------------------------------------------------------------------------------\n");
    printf("     1         India To United states       2,000       100                 46\n");
    printf("     2         India To China               1,500       75                  16\n");
    printf("     3         India To Singapore           4,000       100                 85\n");
}
int choice()
{
}
struct flight{
    int ticket;
    int price;
};

//Initialization of variables
struct flight book[10];
int fare[3][10]={{1500,1000,3000},{1700,2000,4000},{2000,1500,4000}};
int amount=0;
int rfare=0;
int main() {

    //Setting Username=GIT and Password=root
    char user[50], pw[50], adminu[50]="GIT", adminpw[50]="root";
    int trip, date, date2, a, x, y;
    float total;
    char repeat;
    int amount=0;
    printf("\nEnter username: ");
    scanf("%s", &user);
    printf("\nEnter password: ");
    scanf("%s", &pw);
    if (strcmp(user, adminu)==0 && strcmp(pw, adminpw)==0)
    {
        printf("\nWelcome");
        printf("\nChoose if one way trip or roundtrip: ");    // Choosing One way or Round Trip.
        printf("\n1. One way trip \n2. Roundtrip\n");
        scanf("\n%d", &trip);
        printf("\nDate: ");
        printf("\n1. November 23, 2019 \n2. November 24, 2019\n3. November 25, 2019(not applicable for round trip)\n");
        scanf("%d", &date);
        //Selection of three different dates and depending on it choosing the flights.One Way Trip
        switch (trip)
        {
        case 1:
        {
            if (date==1)
            {
                printf("\nNovember 23, 2019\n");
                option1();
                printf("\nHow many tickets will you get?:");
                scanf("%d", &x);
                for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                }

            }

            else if (date==2)
            {
            printf("\nNovember 24, 2019\n");
            option2();
            printf("\nHow many tickets will you get?:  ");
            scanf("%d", &x);
            for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                }
            }
            else if (date==3)
            {
                printf("\nANovember 25, 2019\n");
                option3();
                printf("\nHow many tickets will you get?: ");
                scanf("%d", &x);
                for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                }
            }
            else printf("\nInvalid");
            break;
        }
        //Round Trip implementation using case statement
        case 2:
            {
            if (date==1)
            {
                printf("\nNovember 23, 2019\n");
                option1();
                printf("\nChoose date of return: \n1. November 24, 2019\n2. November 25, 2019\n");
                scanf("%d", &date2);
                printf("\nHow many tickets will you get (excluding the return trip)?: ");
                scanf("%d", &x);
                printf("\nTickets for return trip: %d",x);
                for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                    if (date2==1)
                    {
                    switch (book[a].ticket)
                    {
                        case 1:
                        {
                            printf("\nDetails for return trip");
                            header();
                            printf("  United States to India      1,700       100             \t45\n");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        case 2:
                        {
                            printf("\nDetails for return trip");
                            header();
                            printf("  China to India           2,000        75               \t67\n");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        case 3:
                        {
                            printf("\nDetails for return trip");
                            header();
                            printf("  Singapore to India        4,000       100               \t46\n");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        default: break;
                    }
                    }
                    else if (date2==2)
                    {
                        switch (book[a].ticket)
                    {
                        case 1:
                        {
                            header();
                            printf("  United States to India         2,000      100         \t45");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        case 2:
                        {
                            header();
                            printf("  China to India                1,500        75          \t67");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        case 3:
                        {
                            header();
                            printf("  Singapore to India             4,000       100         \t46");
                            rfare=rfare+fare[date][book[a].ticket-1];
                            break;
                        }
                        default: break;
                    }
                    }
                    else
                    {
                    printf("\nInvalid");
                    return 0;
                    }
                }
            }
            else if (date==2)
            {
            printf("\nNovember 24, 2019\n");
            option2();
            printf("\nHow many tickets will you get?: ");
            scanf("%d", &x);
            for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                }
            }
            else if (date==3)
            {
                printf("\nNovember 25, 2019\n");
                option3();
                printf("\nHow many tickets will you get?: ");
                scanf("%d", &x);
                for (a=1; a<=x; a++)
                {
                    printf("\nPlease select which flight you will book: ");
                    scanf("%d", &book[a].ticket);
                    amount=amount+fare[date-1][book[a].ticket-1];
                }
            }
            else printf("\nInvalid");
            break;
                break;
            }
            default:
             break;
        }
        printf("\n");
    //Total Bill
    printf("\nSummary: ");
    for (a=1; a<=x; a++)
    {
    printf("\nDetails of Ticket no. [%d]", a);
    printf("\nFlight no. [%d]\n", book[a].ticket);
    }
    total=book[a].price;
    printf("\nTotal number of tickets: %d\n", x);
    printf("\nTotal Price for one way trip: %d",amount);
    printf("\nTotal Price for return trip: %d\n",rfare);
    printf("\nThank you for booking..........Have a safe journey.\n");
}
    else
    {
    printf("\nInvalid login\n");
    }
    return 0;
}