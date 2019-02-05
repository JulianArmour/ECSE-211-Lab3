package ca.mcgill.ecse211.lab3;

/**
 * <ul>
 *  <li>start: The very beginning of the program. The robot hasn't reached any waypoint</li>
 *  <li>navigating: The robot is traveling in a straight trajectory towards a waypoint</li>
 *  <li>avoiding: The robot has detected an obstacle and it is moving around it</li>
 *  <li>atWaypoint: The robot has arrived at one of the waypoints</li>
 *  <li>rotating: The robot is rotating towards a specific direction</li>
 *  <li>end: The robot has reached the final waypoint</li>
 * </ul>
 * 
 * @author Julian Armour, Alice Kazarine
 *
 */
public enum NavigatorState {
start, navigating, avoiding, atWaypoint, rotating, end;
}
