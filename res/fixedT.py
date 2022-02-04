# Calculation of fixed transformtion
import numpy as np 

cs0 = np.array([-30.3521,24.3051,38.9612])
cs1 = np.array([-30.3521,-25.6949,38.9612])
cs2 = np.array([19.6458,24.3051,38.4987])
cs3 = np.array([19.6458,-25.6949,38.4987])

xVect = cs3-cs2
yVect = cs2-cs0

xMag = np.linalg.norm(xVect)
yMag = np.linalg.norm(yVect)

unitX = xVect/xMag
unitY = yVect/yMag
unitZ = np.cross(unitX,unitY)

print("UnitX",unitX)
print("UnitY",unitY)
print("UnitZ",unitZ)

RT = np.zeros(shape=(3,3))

RT[:,0]= unitX
RT[:,1]= unitY
RT[:,2]= unitZ 

print("RT",RT)