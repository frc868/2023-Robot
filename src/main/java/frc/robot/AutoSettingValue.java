package frc.robot;

import com.techhounds.houndutil.houndauto.BaseAutoSettingValue;

public enum AutoSettingValue implements BaseAutoSettingValue {
    YES("YES"),
    NO("NO"),
    HYBRID("HYBRID"),
    MID("MID"),
    HIGH("HIGH"),
    CONE("CONE"),
    CUBE("CUBE"),
    ITEM1("ITEM1"),
    ITEM2("ITEM2"),
    ITEM3("ITEM3"),
    ITEM4("ITEM4"),
    A("A"),
    B("B"),
    C("C"),
    D("D"),
    E("E"),
    F("F"),
    G("G"),
    H("H"),
    I("I"),
    CUBAPULT("CUBAPULT"),
    NONE("NONE");

    private final String name;

    private AutoSettingValue(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }
}
