# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# tooldemo.thrift

service tooldemo_IDL
{
    /**
     * Select the best tool.
     * @return true/false on success/failure
     */
    bool select();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
