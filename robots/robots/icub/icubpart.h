#ifndef ICUBPART_H
#define ICUBPART_H

#include <string>
#include <mutex>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <utilities/vector.h>


class iCubPart {

public:
    virtual void activate(const std::string& robotname, const std::string& localname) = 0;

    virtual void deactivate() = 0;

    virtual void refresh() = 0;

    virtual std::string name() = 0;

};


class DriverPart : public iCubPart {

public:

    // just sets the encoders size, should be called by subclasses
    virtual void activate(const std::string&, const std::string&) override;

    virtual void deactivate() override;

    // encoders
    virtual void refresh() override;
    virtual unsigned int dof() const = 0;
    virtual Utils::Vector encodersPos() const;
    virtual Utils::Vector encodersVel() const;

    // move
    virtual void movePos(const Utils::Vector& refs, bool wait = false);
    virtual void moveVel(const Utils::Vector& refs, bool wait = false);
    virtual void movePosJoint(unsigned int joint, double ref, bool wait = false);
    virtual void moveVelJoint(unsigned int joint, double ref, bool wait = false);
    void setControlMode(const int mode);
    void home();

    // limits
    virtual Utils::Vector getMinPos() const = 0;
    virtual Utils::Vector getMaxPos() const = 0;
    virtual Utils::Vector getMinVel() const = 0;
    virtual Utils::Vector getMaxVel() const = 0;
    Utils::Vector getInitialPosition() const;
    void setInitialPosition(const Utils::Vector& initpos);

protected:

    // stored encoders
    Utils::Vector pos;
    Utils::Vector vel;
    Utils::Vector initpos;
    mutable std::mutex mtx;

    yarp::dev::PolyDriver driver;

    Utils::Vector trimToLimitsPos(const Utils::Vector& refs) const;
    Utils::Vector trimToLimitsVel(const Utils::Vector& refs) const;
    double trimToLimitsPosJoint(unsigned int j, double ref) const;
    double trimToLimitsVelJoint(unsigned int j, double ref) const;
    static Utils::Vector trim(const Utils::Vector& refs, const Utils::Vector& min, const Utils::Vector& max);
    static double trimjoint(double ref, double min, double max);

};


class _Inertial : public iCubPart {

public:

    virtual void activate(const std::string& robotname, const std::string& localname) override;
    virtual void deactivate() override;

    virtual void refresh() override;

    virtual Utils::Vector rotationsPos() const;
    virtual Utils::Vector rotationsVel() const;

protected:
    // stored data
    Utils::Vector rotpos;
    Utils::Vector rotvel;
    mutable std::mutex mtx;

    yarp::os::BufferedPort<yarp::sig::Vector> port;

};

//////////////////////
// common driver parts

class _iCubHead : public DriverPart {

public:
    virtual unsigned int dof() const override;

};

class _iCubTorso : public DriverPart {

public:
    virtual unsigned int dof() const override;

};

class _iCubRightArm : public DriverPart {
public:
    virtual unsigned int dof() const override;

    virtual void refresh() override;

    virtual void movePos(const Utils::Vector& refs, bool wait = false);
    virtual void moveVel(const Utils::Vector& refs, bool wait = false);

private:
    static Utils::Vector handposition;

};


//////////////////////
/// traits
///
class HasHead {

public:
    _iCubHead* head = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasTorso {

public:
    _iCubTorso* torso = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasRightArm {

public:
    _iCubRightArm* rightarm = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasInertial {

public:
    _Inertial* inertial = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

#endif // ICUBPART_H
