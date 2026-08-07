/**
 * Live Map page — §6.2
 *
 * Side by side: the polar LIDAR plot (distance vs. angle) and the occupancy grid
 * from slam_toolbox with pose, trail and Nav2 plan overlaid.
 */

import { useBot } from '../lib/store.jsx'
import OccupancyMap from '../components/OccupancyMap.jsx'
import PolarScan from '../components/PolarScan.jsx'

export default function LiveMap() {
  const { status, nodemcu, clearTrail, connected, controlMode } = useBot()
  const rosAvailable = status?.ros?.available

  return (
    <>
      {!nodemcu?.connected && connected && (
        <div className="notice warn">
          <strong>NodeMCU not connected.</strong>
          No LIDAR data is arriving. Check the ESP8266's WiFi credentials and
          backend IP, and remember its serial console is unusable while the
          RPLIDAR holds the hardware UART — its debug output appears on the Logs
          page instead.
        </div>
      )}

      {!rosAvailable && (
        <div className="notice info">
          <strong>ROS2 is not running.</strong>
          The polar scan below works regardless, but the occupancy grid needs
          slam_toolbox. Start it with{' '}
          <span className="mono">ros2 launch slam_bot_bringup bringup.launch.py</span>.
        </div>
      )}

      <div className="grid grid-2">
        <div className="panel">
          <div className="panel-title">
            LIDAR scan — distance &amp; angle
            <span className="small faint">RPLIDAR A1M8, robot frame</span>
          </div>
          <PolarScan height={430} />
        </div>

        <div className="panel">
          <div className="panel-title">
            Occupancy grid — /map
            <button className="sm" onClick={clearTrail} disabled={!connected}>
              Clear trail
            </button>
          </div>
          <OccupancyMap height={430} />
        </div>
      </div>

      <div className="panel">
        <div className="panel-title">Navigation</div>
        <div className="row">
          <span className={`chip ${controlMode === 'nav2' ? 'ok' : 'warn'}`}>
            mode: {controlMode}
          </span>
          <span className="chip">
            plan: {status?.map_meta?.width ? 'map available' : 'no map'}
          </span>
          {status?.ros?.nodes?.length > 0 && (
            <span className="chip">ROS nodes: {status.ros.nodes.join(', ')}</span>
          )}
        </div>
        <div className="param-help" style={{ marginTop: 9 }}>
          Click anywhere on the occupancy grid to pick a goal, then confirm to
          send it to Nav2 (SmacPlanner2D global planner + DWB local controller,
          per §8). Goals are only accepted while the bot is Started and in Nav2
          mode, and are bounds-checked before reaching the planner. The
          Dashboard's Stop and the E-STOP button both cancel an active goal.
        </div>
      </div>
    </>
  )
}
