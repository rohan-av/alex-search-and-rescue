#include <ncurses.h>
#include <iostream>

int main(){
        initscr();
        noecho();
        int c = getch();
        while (c!='q'){
                c = getch();
                std::cout << c << std::endl;
        }
        endwin();
        clear();
}
