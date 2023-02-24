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

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

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

    static class TestPovDirSupplier implements IntSupplier {
        private int direction;

        public void setDirection(int direction) { this.direction = direction; }
        @Override
        public int getAsInt() {
            return direction;
        }
    }

    static class TestGrabber extends Grabber {
        private double speed = 0.0;
        double velocity = 20;
        public TestGrabber() {
            super(true);
        }

        public double getGrabberSpeed() { return speed; }
        public void setGrabberSpeed(double percentage) { speed = percentage; }

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

            povDir.setDirection(180);
            testCollectCommand.execute();
            assertEquals(CollectCommand.MovementState.EJECT, testCollectCommand.movementState());

            povDir.setDirection(0);
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