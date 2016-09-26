#ifndef LIMBGROUP_H
#define LIMBGROUP_H
#include "ofMain.h"
#include "limb.h"


class limbGroup
{
    public:
        /** Default constructor */
        limbGroup();
        /** Default destructor */
        virtual ~limbGroup();

        vector<Limb*> group;

        bool sideChosen = false;
    protected:
    private:
};

#endif // LIMBGROUP_H
