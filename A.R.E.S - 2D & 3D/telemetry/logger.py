import csv
import os
import time


class TelemetryLogger:

    def __init__(self):

        # =============================================
        # TELEMETRY DATA
        # =============================================

        self.data = []

        self.start_time = time.time()

        self.last_log_time = 0.0

        self.log_interval = 0.1


    # =============================================
    # RECORD DATA
    # =============================================

    def record(
        self,
        rover,
        battery,
        mission
    ):

        current_time = (
            time.time()
            - self.start_time
        )


        # Avoid recording too frequently

        if (
            current_time
            - self.last_log_time
            < self.log_interval
        ):

            return


        self.last_log_time = current_time


        # =========================================
        # TELEMETRY SAMPLE
        # =========================================

        sample = {

            "time": current_time,

            "x": rover.x,

            "y": rover.y,

            "theta": rover.theta,

            "left_wheel": rover.v_left,

            "right_wheel": rover.v_right,

            "battery": battery.percentage(),

            "target_error":
                rover.get_distance_to_target(),

            "mission_distance":
                mission.total_distance,

            "waypoints":
                len(rover.path),

            "mission_status":
                mission.status

        }


        self.data.append(
            sample
        )


    # =============================================
    # SAVE CSV
    # =============================================

    def save_csv(
        self,
        filename="reports/telemetry.csv"
    ):

        # Create directory if necessary

        directory = os.path.dirname(
            filename
        )


        if directory:

            os.makedirs(
                directory,
                exist_ok=True
            )


        # =========================================
        # NOTHING TO SAVE
        # =========================================

        if not self.data:

            return


        # =========================================
        # CSV COLUMNS
        # =========================================

        fieldnames = [

            "time",

            "x",

            "y",

            "theta",

            "left_wheel",

            "right_wheel",

            "battery",

            "target_error",

            "mission_distance",

            "waypoints",

            "mission_status"

        ]


        # =========================================
        # WRITE CSV
        # =========================================

        with open(
            filename,
            "w",
            newline=""
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames
            )


            writer.writeheader()


            writer.writerows(
                self.data
            )


    # =============================================
    # NUMBER OF SAMPLES
    # =============================================

    def sample_count(self):

        return len(
            self.data
        )


    # =============================================
    # CLEAR DATA
    # =============================================

    def clear(self):

        self.data.clear()

        self.start_time = time.time()

        self.last_log_time = 0.0