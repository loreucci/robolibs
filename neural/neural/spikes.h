#ifndef SPIKES_H
#define SPIKES_H

#include <vector>
#include <mutex>
#include <memory>

#include <sec/controller.h>


namespace neural {


struct Spike {
    unsigned int neuron_id;
    double time;
};


class SpikeData : public std::vector<Spike> {

public:
    using std::vector<Spike>::vector;

    void append(const SpikeData& data);

};



class SpikeNodeOut;


class SpikeNodeIn {

public:
    SpikeNodeIn(const SpikeNodeOut* nodeout = nullptr);

    virtual void connect(const SpikeNodeOut* nodeout);

    bool isConnected() const;

    void pushData(const SpikeData& newdata, unsigned int newdataid);

    void refreshData();

    operator SpikeData() const;

    SpikeData getData() const;

    bool isNew() const;

protected:
    std::shared_ptr<std::mutex> mtx;
    const SpikeNodeOut* nodeout;
    SpikeData data, buffereddata;
    unsigned int lastid, bufferedid;
    bool isnew;

};


class SpikeNodeOut {

public:
    SpikeNodeOut();

    SpikeNodeOut& operator=(const SpikeData& data);

    void addData(const SpikeData& data);

    std::pair<SpikeData, unsigned int> getData() const;

    void addConnection(SpikeNodeIn* link);

protected:
    SpikeData data;
    unsigned int dataid;
    std::vector<SpikeNodeIn*> links;

};

}


namespace sec {

// this will not register the connection,
// but it may be useful if the links are not
// in nodes
void connect(neural::SpikeNodeOut* out, neural::SpikeNodeIn* in);

template <class C1, class C2>
void connect(C1& source, neural::SpikeNodeOut C1::* out, C2& sink, neural::SpikeNodeIn C2::* in) {

    (sink.*in).connect(&(source.*out));
    (source.*out).addConnection(&(sink.*in));
    main_controller.registerConnection(&source, &sink);

}

}

#endif // SPIKES_H
