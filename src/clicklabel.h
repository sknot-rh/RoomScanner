#ifndef CLICKLABEL_H
#define CLICKLABEL_H

#include <QLabel>
#include <QMainWindow>
#include <QThread>
#include <boost/thread/thread.hpp>

class clickLabel : public QLabel
{
    Q_OBJECT
public:
    boost::thread* thr;
    clickLabel( boost::thread* thr, QWidget * parent = 0 );
    ~clickLabel(){}

signals:
    void labelClicked();

public slots:
    void slotClicked();

protected:
    void mousePressEvent ( QMouseEvent * event ) ;

};

#endif // CLICKLABEL_H
