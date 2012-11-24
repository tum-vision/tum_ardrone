#pragma once
/*
 * Author: Jakob Engel <jajuengel@gmail.com>
 */
 
#include <QtGui/QWidget>
#include "ui_tum_ardrone_gui.h"
#include "geometry_msgs/Twist.h"

class RosThread;
class PingThread;
struct ControlCommand;

enum ControlSource {CONTROL_KB = 0, CONTROL_JOY = 1, CONTROL_AUTO = 2, CONTROL_NONE = 3};

class tum_ardrone_gui : public QWidget
{
    Q_OBJECT

public slots:
	void LandClicked();
	void TakeoffClicked();
	void ToggleCamClicked();
	void EmergencyClicked();

	void ClearClicked();
	void SendClicked();
	void ClearSendClicked();
	void ResetClicked();
	void FlattrimClicked();

	void LoadFileChanged(QString val);
	void ToggledUseHovering(int val);
	void ToggledPingDrone(int val);

	void ControlSourceChanged();

private slots:
    void setCountsSlot(unsigned int nav,unsigned int control,unsigned int pose,unsigned int joy);
    void setPingsSlot(int p500, int p20000);
    void setControlSourceSlot(int cont);

    void addLogLineSlot(QString);
    void setAutopilotInfoSlot(QString);
    void setStateestimationInfoSlot(QString);

    void closeWindowSlot();


signals:
	void setCountsSignal(unsigned int nav,unsigned int control,unsigned int pose,unsigned int joy);
    void setPingsSignal(int p500, int p20000);
	void setControlSourceSignal(int cont);

	void addLogLineSignal(QString);
	void setAutopilotInfoSignal(QString);
	void setStateestimationInfoSignal(QString);

	void closeWindowSignal();


public:
    tum_ardrone_gui(QWidget *parent = 0);
    ~tum_ardrone_gui();
    RosThread* rosThread;
    PingThread* pingThread;

    void setCounts(unsigned int nav,unsigned int control,unsigned int pose,unsigned int joy);
    void setPings(int p500, int p20000);
    void setControlSource(ControlSource cont);
    void addLogLine(std::string s);
    void setAutopilotInfo(std::string s);
    void setStateestimationInfo(std::string s);
    void closeWindow();

    // calculates KB command, based on currently pressed keys.
    ControlCommand calcKBControl();
    ControlSource currentControlSource;
    double sensGaz, sensYaw, sensRP;
    bool useHovering;

protected:

    // keyboard control.... this is the only way i managed to do this...
    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);
    int mapKey(int k);
    bool isPressed[8];	//{j k l i u o q a}
    unsigned int lastRepeat[8];


private:
    Ui::tum_ardrone_guiClass ui;
};


