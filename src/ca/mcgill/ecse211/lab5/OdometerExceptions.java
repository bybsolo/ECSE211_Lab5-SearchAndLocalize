package ca.mcgill.ecse211.lab5;

/**
 * This class is used to handle errors regarding the singleton pattern used for the odometer and odometerData
 * @author Team 12
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

  public OdometerExceptions(String Error) {
    super(Error);
  }

}
