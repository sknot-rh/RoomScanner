#include "application.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
    QApplication a (argc, argv);
    RoomScanner w;
    w.show ();
    return a.exec ();
}
