#include "application.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
    QApplication a (argc, argv);
    RoomScanner w;
    QObject::connect(&a, SIGNAL(aboutToQuit()), &w, SLOT(closing()));
    w.show ();
    return a.exec ();
}