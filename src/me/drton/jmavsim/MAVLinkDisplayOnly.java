package me.drton.jmavsim;

import me.drton.jmavlib.conversion.RotationConversion;
import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;
import me.drton.jmavsim.vehicle.AbstractVehicle;

import gov.nasa.xpc.XPlaneConnect;
import java.io.IOException;
import java.net.SocketException;

import javax.vecmath.*;
import java.util.Arrays;

/**
 * MAVLinkDisplayOnly is MAVLink bridge between AbstractVehicle and autopilot connected via MAVLink.
 * MAVLinkDisplayOnly should have the same sysID as the autopilot, but different componentId.
 * It reads HIL_STATE_QUATERNION from the MAVLink and displays the vehicle position and attitude.
 * @author Romain Chiappinelli
 */
public class MAVLinkDisplayOnly extends MAVLinkHILSystemBase {

    private boolean firstMsg=true;       // to detect the first MAVLink message
    private double lat0;                // initial latitude (radians)
    private double lon0;                // initial longitude (radians)
    private double alt0;                // initial altitude (meters)
    private double lat;                 // geodetic latitude (radians)
    private double lon;                 // geodetic longitude (radians)
    private double lat_deg;             // geodetic latitude (degrees)
    private double lon_deg;             // geodetic longitude (degrees)
    private double alt;                 // above sea level (meters)
    private float roll_deg, pitch_deg, yaw_deg; // euler angles in degrees (to represent vehicle attitude)
    private float gears_out=1.0f;       // set to 0 to display the landing gears inside in X plane
    private double [] quat={0.0,0.0,0.0,0.0};   // unit quaternion for attitude representation
    private Double [] control = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    private static final double EARTH_RADIUS=6371000.0;    // earth radius in meters
    private XPlaneConnect xpc=null;

    /**
     * Create MAVLinkDisplayOnly, MAVLink system that sends nothing to autopilot and passes states from
     * autopilot to simulator
     *
     * @param sysId       SysId of simulator should be the same as autopilot
     * @param componentId ComponentId of simulator should be different from autopilot
     * @param vehicle     vehicle to connect
     */
    public MAVLinkDisplayOnly(MAVLinkSchema schema, int sysId, int componentId, AbstractVehicle vehicle) {
        super(schema, sysId, componentId, vehicle);
    }

    @Override
    public void handleMessage(MAVLinkMessage msg) {
        if ("HIL_STATE_QUATERNION".equals(msg.getMsgName())) {

            if (firstMsg) {
                firstMsg=false;
                // we take the first received position as initial position
                lat0=Math.toRadians(msg.getDouble("lat")*1e-7);
                lon0=Math.toRadians(msg.getDouble("lon")*1e-7);
                alt0=msg.getDouble("alt")*1e-3;
            }
            for (int i = 0; i < 4; ++i) {
                quat[i] = ((Number)((Object[])msg.get("attitude_quaternion"))[i]).doubleValue();
            }
            lat_deg=msg.getDouble("lat")*1e-7;
            lon_deg=msg.getDouble("lon")*1e-7;
            lat=Math.toRadians(lat_deg);
            lon=Math.toRadians(lon_deg);
            alt=msg.getDouble("alt")*1e-3;

            Vector3d pos = new Vector3d(EARTH_RADIUS*(lat-lat0),2.0+EARTH_RADIUS*(lon-lon0)*Math.cos(lat0),alt0-alt);
            double [] euler = RotationConversion.eulerAnglesByQuaternion(quat);
            Matrix3d dcm = new Matrix3d(RotationConversion.rotationMatrixByEulerAngles(euler[0],euler[1],euler[2]));
            roll_deg=(float)Math.toDegrees(euler[0]);
            pitch_deg=(float)Math.toDegrees(euler[1]);
            yaw_deg=(float)Math.toDegrees(euler[2]);

            vehicle.setControl(Arrays.asList(control));     // set 0 throttles
            vehicle.setPosition(pos);   // we want ideally a "local" pos groundtruth
            vehicle.setRotation(dcm);

            // send the vehicle position and orientation to Xplane via Xplane connect
            try {
                if (xpc==null) {
                    xpc = new XPlaneConnect();
                    // Ensure connection established.
                    xpc.getDREF("sim/test/test_float");
                    // disable the x-plane physics engine so the vehicle won't fall
                    // xpc.sendDREF("sim/operation/override/override_planepath",1.0f);
                    // xpc.pauseSim(true);   // pausing X plane simulator engine
                    // xpc.sendCTRL(ctrl);
                }
                double[] posi = new double[] {lat_deg, lon_deg, alt, pitch_deg, roll_deg, yaw_deg};
                // float[] posi = new float[] {37.524F, -122.06899F, 2500, 0, 0, 0, 1};
                xpc.sendPOSI(posi);
            } catch (SocketException ex) {
                System.out.println("Unable to set up the connection. (Error message was '" + ex.getMessage() + "'.)");
            } catch (IOException ex) {
                System.out.println("Something went wrong with one of the commands. (Error message was '" + ex.getMessage() + "'.)");
            }
        }
    }

    @Override
    public boolean gotHilActuatorControls()
    {
        return !firstMsg;
    }

    @Override
    public void initMavLink()
    {
        firstMsg=true;
    }

    @Override
    public void endSim()
    {

    }

}
