/* Wheelbox.cpp */
/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

	Wheelbox widget is based in part on the work of the Qwt project(http://qwt.sf.net).

*/

#include "wheelbox.h"
#include <qwt_wheel.h>
#include <qlcdnumber.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qevent.h>
#include <qapplication.h>

class Wheel: public QwtWheel
{
public:
    Wheel( WheelBox *parent ):
        QwtWheel( parent )
    {
        setFocusPolicy( Qt::WheelFocus );
        parent->installEventFilter( this );
    }

    virtual bool eventFilter( QObject *object, QEvent *event )
    {
        if ( event->type() == QEvent::Wheel )
        {
            const QWheelEvent *we = static_cast<QWheelEvent *>( event );

            QWheelEvent wheelEvent( QPoint( 5, 5 ), we->delta(),
                we->buttons(), we->modifiers(),
                we->orientation() );

            QApplication::sendEvent( this, &wheelEvent );
            return true;
        }
        return QwtWheel::eventFilter( object, event );
    }
};

WheelBox::WheelBox( const QString &title,
        double min, double max, double stepSize, QWidget *parent ):
    QWidget( parent )
{

    d_number = new QLCDNumber( this );
    d_number->setSegmentStyle( QLCDNumber::Filled );
    d_number->setAutoFillBackground( true );
    d_number->setFixedHeight( d_number->sizeHint().height() * 2 );
    d_number->setFocusPolicy( Qt::WheelFocus );

    QPalette pal( Qt::black );
    pal.setColor( QPalette::WindowText, Qt::green );
    d_number->setPalette( pal );

    d_wheel = new Wheel( this );
    d_wheel->setOrientation( Qt::Vertical );
    d_wheel->setInverted( true );
    d_wheel->setRange( min, max );
    d_wheel->setSingleStep( stepSize );
    d_wheel->setPageStepCount( 5 );
    d_wheel->setFixedHeight( d_number->height() );

    d_number->setFocusProxy( d_wheel );

    QFont font( "Helvetica", 10 );
    font.setBold( true );

    d_label = new QLabel( title, this );
    d_label->setFont( font );

    QHBoxLayout *hLayout = new QHBoxLayout;
    hLayout->setContentsMargins( 0, 0, 0, 0 );
    hLayout->setSpacing( 2 );
    hLayout->addWidget( d_number, 10 );
    hLayout->addWidget( d_wheel );

    QVBoxLayout *vLayout = new QVBoxLayout( this );
    vLayout->addLayout( hLayout, 10 );
    vLayout->addWidget( d_label, 0, Qt::AlignTop | Qt::AlignHCenter );

    connect( d_wheel, SIGNAL( valueChanged( double ) ),
        d_number, SLOT( display( double ) ) );
    connect( d_wheel, SIGNAL( valueChanged( double ) ),
        this, SIGNAL( valueChanged( double ) ) );
}

void WheelBox::setTheme( const QColor &color )
{
    d_wheel->setPalette( color );
}

QColor WheelBox::theme() const
{
    return d_wheel->palette().color( QPalette::Window );
}

void WheelBox::setValue( double value )
{
    d_wheel->setValue( value );
    d_number->display( value );
}

double WheelBox::value() const
{
    return d_wheel->value();
}
