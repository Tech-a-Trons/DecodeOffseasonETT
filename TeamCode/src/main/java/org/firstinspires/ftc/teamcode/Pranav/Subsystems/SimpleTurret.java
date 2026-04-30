package org.firstinspires.ftc.teamcode.Pranav.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SimpleTurret implements Subsystem {
    public static final SimpleTurret INSTANCE = new SimpleTurret();
    private SimpleTurret() { }
    private ServoEx leftservo = new ServoEx("leftservo");
    private ServoEx rightservo = new ServoEx("righttservo");
    public Command left1 = new SetPosition(leftservo, 0.5).requires(this);
    public Command left2 = new SetPosition(leftservo, 0).requires(this);
    public Command right1 = new SetPosition(rightservo, 0.5).requires(this);
    public Command right2 = new SetPosition(rightservo, 0).requires(this);

}
