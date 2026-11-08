// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563;

import br.com.megazord7563.util.geometry.GeomUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class XBbotMechanism3d {
  private static XBbotMechanism3d instance;

  @Getter @Setter public double L1Height = 0.0;
  @Getter @Setter public double L2Height = 0.0;
  @Getter @Setter public double L3Height = 0.0;

  public static XBbotMechanism3d getInstance() {
    if (instance == null) {
      instance = new XBbotMechanism3d();
    }
    return instance;
  }

  public void log() {
    var pose_level3 = new Pose3d(new Translation3d(0.0, 0.0, L1Height), new Rotation3d());
    var pose_level2 = new Pose3d(new Translation3d(0.0, 0.0, L2Height), new Rotation3d());
    var pose_level1 = new Pose3d(new Translation3d(0.0, 0.0, L3Height), new Rotation3d());

    Logger.recordOutput("Mechanism3d/Components", pose_level1, pose_level2, pose_level3);
  }
}
