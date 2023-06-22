#!/bin/bash

mods="\
    mrpt \
    mrpt.ros_bridge \
    mrpt.pymrpt \
    mrpt.pymrpt.mrpt \
    mrpt.pymrpt.mrpt.apps \
    mrpt.pymrpt.mrpt.bayes \
    mrpt.pymrpt.mrpt.comms \
    mrpt.pymrpt.mrpt.config \
    mrpt.pymrpt.mrpt.containers \
    mrpt.pymrpt.mrpt.cpu \
    mrpt.pymrpt.mrpt.expr \
    mrpt.pymrpt.mrpt.global_settings \
    mrpt.pymrpt.mrpt.graphs \
    mrpt.pymrpt.mrpt.gui \
    mrpt.pymrpt.mrpt.hwdrivers \
    mrpt.pymrpt.mrpt.img \
    mrpt.pymrpt.mrpt.io \
    mrpt.pymrpt.mrpt.kinematics \
    mrpt.pymrpt.mrpt.maps \
    mrpt.pymrpt.mrpt.math \
    mrpt.pymrpt.mrpt.nav \
    mrpt.pymrpt.mrpt.obs \
    mrpt.pymrpt.mrpt.opengl \
    mrpt.pymrpt.mrpt.poses \
    mrpt.pymrpt.mrpt.random \
    mrpt.pymrpt.mrpt.rtti \
    mrpt.pymrpt.mrpt.serialization \
    mrpt.pymrpt.mrpt.slam \
    mrpt.pymrpt.mrpt.system \
    mrpt.pymrpt.mrpt.tfest \
    mrpt.pymrpt.mrpt.topography \
    mrpt.pymrpt.mrpt.typemeta \
    mrpt.pymrpt.mrpt.vision \
"

for MOD in $mods; do
    echo "Generating pydoc html for $MOD..."
    pydoc3 -w $MOD
done

tree -H '.' -L 1 --noreport --charset utf-8 -o index.html

