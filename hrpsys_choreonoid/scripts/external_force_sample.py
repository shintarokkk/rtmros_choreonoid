import cnoid.Base
import cnoid.BodyPlugin
import numpy as np

def addExternalForce():
    robotname = "JAXON_RED"
    linkname = "RARM_JOINT7"
    pos   = [0, 0, 0.2] ## link local position
    rarot = np.matrix(np.array([[0.022978, -0.934776, -0.354494], [0.945668, -0.094707, 0.311034], [-0.32432, -0.342381, 0.881812]]))
    force = np.dot(rarot, np.array([-20, 0, 0])) ## [ N ]
    force = np.ravel(force)
    tm = 30 ## [ sec ]

    robotItem = cnoid.Base.RootItem.instance().findItem(robotname)
    simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
    pushingLink = robotItem.body().link(linkname)
    if pushingLink:
        simulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)

addExternalForce()
