package org.tahomarobotics.robot.grabber;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static org.junit.jupiter.api.Assertions.*;
import static org.tahomarobotics.robot.grabber.GrabberConstants.TRIGGER_DEAD_ZONE;
import static org.mockito.Mockito.mockStatic;
class CollectCommandTest {
    private static final Logger logger = LoggerFactory.getLogger(CollectCommandTest.class);
    class TestDriveLeftTrigerSupplier implements DoubleSupplier {
        private double position;

        public void greaterThanDeadZone() {
            position = TRIGGER_DEAD_ZONE+1.0;
        }

        public void lessThanDeadZone() {
            position = TRIGGER_DEAD_ZONE-1.0;
        }

        @Override
        public double getAsDouble() {
            return position;
        }
    }

    class TestManipLeftTrigerSupplier implements DoubleSupplier {
        private double position;

        public void greaterThanDeadZone() {
            position = TRIGGER_DEAD_ZONE+1.0;
        }

        public void lessThanDeadZone() {
            position = TRIGGER_DEAD_ZONE-1.0;
        }

        @Override
        public double getAsDouble() {
            return position;
        }
    }

    class TestPovDirSupplier implements IntSupplier {
        private int direction;

        public void set0() {
            direction = 0;
        }
        public void set90() {
            direction = 90;
        }
        public void set180() {
            direction = 180;
        }
        public void set270() {
            direction = 270;
        }
        @Override
        public int getAsInt() {
            return direction;
        }
    }

    private double value = 0.0;
    class TestGrabber extends Grabber {
        double velocity = 20;
        public TestGrabber() {
            super(true);
        }
        public void setGrabberSpeed(double percentage) {
            value = percentage;
            logger.info("Grabber speed: " + percentage);
        }

        public double getVelocity() {
            return velocity;
        }
        public void setVelocity(double v) { velocity =  v; }

    }

    TestDriveLeftTrigerSupplier driveLeftTriger = new TestDriveLeftTrigerSupplier();
    TestManipLeftTrigerSupplier manipLeftTriger = new TestManipLeftTrigerSupplier();
    TestPovDirSupplier povDir = new TestPovDirSupplier();

    @Test
    public void testCommand1() {
        Grabber testGrabber = new TestGrabber();

        try (MockedStatic<Grabber> gFactory = Mockito.mockStatic(Grabber.class)) {
            gFactory.when(Grabber::getInstance).thenReturn(testGrabber);

            CollectCommand testCollectCommand = new CollectCommand(driveLeftTriger, manipLeftTriger, povDir);
            testCollectCommand.initialize();

            driveLeftTriger.greaterThanDeadZone();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.INJEST, testCollectCommand.movementState());

            driveLeftTriger.lessThanDeadZone();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.RETAIN, testCollectCommand.movementState());
        }
    }

    @Test
    public void testCommand2() {
        Grabber testGrabber = new TestGrabber();

        try (MockedStatic<Grabber> gFactory = Mockito.mockStatic(Grabber.class)) {
            gFactory.when(Grabber::getInstance).thenReturn(testGrabber);

            CollectCommand testCollectCommand = new CollectCommand(driveLeftTriger, manipLeftTriger, povDir);
            testCollectCommand.initialize();

            driveLeftTriger.greaterThanDeadZone();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.INJEST, testCollectCommand.movementState());

            povDir.set180();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.EJECT, testCollectCommand.movementState());

            povDir.set0();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.OFF, testCollectCommand.movementState());

        }
    }

    @Test
    public void testCommand3() {
        Grabber testGrabber = new TestGrabber();

        try (MockedStatic<Grabber> gFactory = Mockito.mockStatic(Grabber.class)) {
            gFactory.when(Grabber::getInstance).thenReturn(testGrabber);

            CollectCommand testCollectCommand = new CollectCommand(driveLeftTriger, manipLeftTriger, povDir);
            testCollectCommand.initialize();

            manipLeftTriger.greaterThanDeadZone();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.SCORE, testCollectCommand.movementState());

            manipLeftTriger.lessThanDeadZone();
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.OFF, testCollectCommand.movementState());
        }
    }
}