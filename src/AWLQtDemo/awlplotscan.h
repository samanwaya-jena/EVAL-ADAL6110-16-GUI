/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the CuteApplication of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

#ifndef AWLPLOTSCAN_H
#define AWLPLOTSCAN_H

#include <QWidget>
#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QAction>
#include <QActionGroup>
#include "boost/chrono/system_clocks.hpp"

#include "ReceiverCapture.h"
#include "ui_awlplotscan.h"
#include "DetectionStruct.h"

//#define USE_FPS_AWLPLOTSCAN

QT_BEGIN_NAMESPACE

namespace awl
{

/** \brief Window managing the display of Waveform (or A-Scan) data.
*/
class AWLPlotScan : public QFrame
{
	Q_OBJECT
public:
   /** \brief  Constructor.
    */
	AWLPlotScan(QWidget *parent = 0);

    /** \brief  Destructor.
    */
    ~AWLPlotScan();


    /** \brief  Start the display of the receiver data.
    *           The function is currently empty and kept for consistency purposes with other display classes.
    */
	void start(SensorCoreScope::ReceiverCapture::Ptr inReceiverCapture);
    /** \brief  Stop the display of the receiver data.
        *           The function is currently empty and kept for consistency purposes with other display classes.
     */
    void stop();

    /** \brief  Sets the mask for the voxels that will be actually displayed.
    */
    void setVoxelMask(uint32_t chMask);

    /** \brief  Supply the ID of the receiver for which A-Scan data is displayed.
     */
  void selectReceiver(int receiver);

   /** \brief  Update the A-Scan display following change of data.
     *         Called by the application main loop for updates when new data is available.
     */
  void AScanDataChanged(const SensorCoreScope::AScan::Vector& inData);

    /** \brief  Show/Hide the A-Scan window.
     */
    void ShowAScan(bool show) { showAScan = show; }
  
    
    /** \brief  Set the maximum range displayed in the A-Scan Window.
      */
    void SetMaxRange(float maxRange) { m_maxRange = maxRange; }

    /** \brief  Used by Qt in resizing, provides "ideal" size for the A-Scan Window.
     */
    QSize sizeHint() const;
    /** \brief  Used by Qt in resizing, provides "minimum" acceptable size for the A-Scan Window.
     */
    QSize minimumSizeHint() const;
    /** \brief  Used by Qt in resizing, provides "maximum" size for the A-Scan Window.
     */
    QSize maximumSizeHint() const;

private:
    /** \brief  True when the window is displayed.
   */
  bool showAScan;

  /** \brief  Holder of the A-Scan data that is under display.
    */
  SensorCoreScope::AScan::Vector aScanData;

  /** \brief  Qt user interface definition structure.
    */
	Ui::AWLPlotScanFrame ui;
  /** \brief  Plot all the A-Scans within the A-Scan window.
    */
	void plotAScans(QPainter* p);
    /** \brief  Plot a single A-Scan  within the A-Scan window.
      */
    void PlotAScan(QPainter* p, SensorCoreScope::AScan::Ptr pAscan, int top, int left, int width, int height, float maxRange);
    /** \brief  Display teh A-Scan labels
    */
    void LabelAScan(QPainter* p);

signals:
   
    /** \brief  Event handler for the closing of the window.
    */
	void closed();
protected :
    /** \brief  Event handler for the re-paint.
    */
    void paintEvent(QPaintEvent *p);
    /** \brief  Event handler for the closing of the window.
     */
    void closeEvent(QCloseEvent * event);
    /** \brief  Event handler for the resizing  of the window.
     */
    void resizeEvent(QResizeEvent * event);

    /** \brief  Pointer to the Qpoints used to display the waveform..
    */
    QPoint * m_pPts;
    /** \brief  Quantity of QPoints used to display the waveform.
    */
    int m_numPts;

    /** \brief  Maximum range of the A-Scan display .
     */
    float m_maxRange;

    /** \brief  Mask used to determine which voxels are displayed.
    */
  uint32_t m_chMask;

  /** \brief  ID of the receiver under display.
   */
  int m_selectedReceiver;

  /** \brief  Total number of channels on the receiver under display.
  */
  uint32_t m_nbrCh;

#ifdef USE_FPS_AWLPLOTSCAN
  // FPS
  /** \brief  Frame per second calculator.  Used for performance evaluation purposes.
  */
  boost::chrono::time_point<boost::chrono::high_resolution_clock> m_timeFPS;
  /** \brief  Qty of frames used in the Frame per second calculator.  Used for performance evaluation purposes.
  */
  int nFrames;
  /** \brief  resulting frame per second in the Frame per second calculator.  Used for performance evaluation purposes.
   */
  int FPS;
#endif //USE_FPS_AWLPLOTSCAN
};

} // namespace awl
#endif // AWLPLOTSCAN_H
