import rospy
from avoidance.avoidance_system import AvoidanceSystem
from avoidance.flight_commander import FlightCommander

if __name__ == "__main__":
    # Иниуциализируем ноду
    rospy.init_node('avoidance_node')
    
    # Создаем экземпляры классов для управления и облета препятствий
    commander = FlightCommander()
    avoidance = AvoidanceSystem(commander)
    commander.do_takeoff()
    rospy.spin()