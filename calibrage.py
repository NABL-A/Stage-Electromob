import numpy as np
import time
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

class Calibrage :

    estFait = False

    def __init__(self, IP="10.2.30.60"):

        pass

    def connexion(self):
        self.rtde_c = RTDEControl(self.IP)
        self.rtde_r = RTDEReceive(self.IP)

    def deconnexion(self):
        self.robot_c.disconnect()

    def get_position(self):
        return np.array(self.rtde_r.getActualTCPPose()[:3])

if __name__ == "__main__":

    calibrage = Calibrage()

    if calibrage.estFait == False :

        print("### Calibrage repère local avec RTDE ###")

        input("Place le robot à l'ORIGINE du repère local (P0), puis appuie sur Entrée...")
        P0 = calibrage.get_position()

        input("Place le robot sur l’AXE X local (P1), puis appuie sur Entrée...")
        P1 = calibrage.get_position()

        input("Place le robot sur l’AXE Y local (P2), puis appuie sur Entrée...")
        P2 = calibrage.get_position()

        input("Positionne la boîte et l'injecteur dans le repère ...")
        #changement angle pince 

        input("place le robot sur la pastille de la boîte")
        boite_local = calibrage.get_position

        input("place le robot sur la pastille de la injecteur")
        injecteur_local = calibrage.get_position

        # Définition des axes X, Y et Z
        x_axis = P1 - P0
        x_axis /= np.linalg.norm(x_axis)

        temp_y = P2 - P0
        z_axis = np.cross(x_axis, temp_y)
        z_axis /= np.linalg.norm(z_axis)

        y_axis = np.cross(z_axis, x_axis)

        # Matrice de rotation (repère local en global)
        R = np.column_stack((x_axis, y_axis, z_axis))

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = P0

        #matrice inverse
        T_inv = np.linalg.inv(T)

        repere_box = T_inv @ boite_local
        repere_box_1 =np.array(list(repere_box[:3])+[1])
        repere_injecteur = T_inv @ injecteur_local
        repere_injecteur_1=np.array(list(repere_injecteur[:3]) + [1])


        calibrage.estFait = True
    

    print("Points capturés.")
    print("P0 :", P0)
    print("P1 :", P1)
    print("P2 :", P2)