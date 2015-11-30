#ifndef SOURCE_H
#define SOURCE_H

#include "node.h"

namespace sec {

class Source : public Node {

public:
    using Node::Node;

    virtual void refreshInputs() final override;

    virtual bool connected() const final override;

};

}

#endif // SOURCE_H
