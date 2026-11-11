// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static boolean isReal = true;
  private static ArrayList<BaseStatusSignal> signals = new ArrayList<BaseStatusSignal>();

  static {
    if (Constants.currentMode == Mode.SIM) isReal = false;
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Refresh all StatusSignals at once for optimization (based on 6328's code) TODO: test if this
   * stuff actually works?
   */
  public static void addSignals(BaseStatusSignal... signal) {
    signals.addAll(Set.of(signal));
  }

  public static void refreshAll() {
    if (isReal) BaseStatusSignal.refreshAll(signals);
  }
}
