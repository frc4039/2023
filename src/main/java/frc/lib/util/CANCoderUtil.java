/* BSD 3-Clause License

Copyright (c) 2022, FRC Team 3512
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

/** Sets status frames for the CTRE CANCoder. */
public class CANCoderUtil {
  public enum CCUsage {
    kAll,
    kSensorDataOnly,
    kFaultsOnly,
    kMinimal
  }

  /**
   * This function allows reducing a CANCoder's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 10ms to 255ms.
   *
   * <p>See https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#cancoder for a description
   * of the status frames.
   *
   * @param cancoder The CANCoder to adjust the status frames on.
   * @param usage The status frame feedback to enable. kAll is the default when a CANCoder
   *     isconstructed.
   */
  public static void setCANCoderBusUsage(CANCoder cancoder, CCUsage usage) {
    if (usage == CCUsage.kAll) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kSensorDataOnly) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    } else if (usage == CCUsage.kFaultsOnly) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kMinimal) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    }
  }
}