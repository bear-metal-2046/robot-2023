/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
package org.tahomarobotics.robot.grabber;

import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.tahomarobotics.robot.grabber.GrabberConstants.TRIGGER_DEAD_ZONE;
class CollectCommandTest {
    static class TestDriveLeftTrigerSupplier implements DoubleSupplier {
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

    static class TestManipLeftTrigerSupplier implements DoubleSupplier {
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

    static class TestPovDirSupplier implements BooleanSupplier {
        private boolean active;

        public void setActive(boolean active) { this.active = active; }
        @Override
        public boolean getAsBoolean() {
            return active;
        }
    }

    static class TestGrabber extends Grabber {
        private double speed = 0.0;
        double current = 20;
        double power = 0;

        public TestGrabber() {
            super(true);
        }

        public double getGrabberSpeed() { return speed; }
        public void setGrabberSpeed(double percentage) { speed = percentage; }

        public double getCurrent() {
            return current;
        }

        public double getPower() {
            return power;
        }

        public void setCurrent(double current) { this.current = current; }

        public void setPower(double power) { this.power = power; }
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
            assertEquals(Grabber.MovementState.INGEST, testGrabber.getState());

            driveLeftTriger.lessThanDeadZone();
            testCollectCommand.execute();
            assertEquals(Grabber.MovementState.RETAIN, testGrabber.getState());
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
            assertEquals(Grabber.MovementState.INGEST, testGrabber.getState());

            povDir.setActive(true);
            testCollectCommand.execute();
            assertEquals(Grabber.MovementState.EJECT, testGrabber.getState());

            povDir.setActive(false);
            testCollectCommand.execute();
            assertEquals(Grabber.MovementState.OFF, testGrabber.getState());

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
            assertEquals(Grabber.MovementState.SCORE, testGrabber.getState());

            manipLeftTriger.lessThanDeadZone();
            testCollectCommand.execute();
            assertEquals(Grabber.MovementState.OFF, testGrabber.getState());
        }
    }
}