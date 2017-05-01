#include "clicklabel.h"

clickLabel::clickLabel( boost::thread* thr, QWidget * parent )
:QLabel(parent)
{
    std::cout << "killing thread " << thr->get_id() << "\n";
    this->thr = thr;
    connect( this, SIGNAL( labelClicked() ), this, SLOT( slotClicked() ) );
}

void clickLabel::slotClicked()
{
    // does not work as expected :(
    this->thr->interrupt();
}

void clickLabel::mousePressEvent ( QMouseEvent * event )
{
    emit labelClicked();
}
