# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0
#
# gesturesRenderingEngine.thrift

service GesturesRenderingEngine_IDL
{
  /**
  * Do the "nod" gesture.
  * @return true/false on success/failure
  */
  bool do_nod();

  /**
  * Do the "punch" gesture.
  * @return true/false on success/failure
  */
  bool do_punch();

  /**
  * Do the "look out" gesture.
  * @return true/false on success/failure
  */
  bool do_lookout();

  /**
  * Do the "thumbs up" gesture.
  * @return true/false on success/failure
  */
  bool do_thumbsup();

  /**
  * Do the "thumbs down" gesture.
  * @return true/false on success/failure
  */
  bool do_thumbsdown();

  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();
}
