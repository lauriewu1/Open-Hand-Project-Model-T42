import lib_robotis_mod

def Initialize():
    dyn= USB2Dynamixel_Device('/dev/ttyUSB0')
    P= Robotis_servo( dyn,1 )
    T= Robotis_servo( dyn,11 )
    P.read_encoder()
    T.read_encoder()

Initialize ()

P.move_to_encoder(200)
T.move_to_encoder(200)
