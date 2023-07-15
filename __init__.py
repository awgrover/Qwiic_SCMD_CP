import busio
import board

# Define the device name and I2C addresses
_DEFAULT_NAME = "Qwiic Serial Control Motor Driver"
_AVAILABLE_I2C_ADDRESS = [0x5D, 0x58, 0x59, 0x5A, 0x5C]

class SCMDDiagnostics:
    def __init__(self):
        self.numberOfSlaves = 0
        self.U_I2C_RD_ERR = 0
        self.U_I2C_WR_ERR = 0
        self.U_BUF_DUMPED = 0
        self.E_I2C_RD_ERR = 0
        self.E_I2C_WR_ERR = 0
        self.LOOP_TIME = 0
        self.SLV_POLL_CNT = 0
        self.MST_E_ERR = 0
        self.MST_E_STATUS = 0
        self.FSAFE_FAULTS = 0
        self.REG_OOR_CNT = 0
        self.REG_RO_WRITE_CNT = 0

class QwiicScmd:
    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    def __init__(self, address=None, i2c_driver=None):
        self.address = address if address is not None else self.available_addresses[0]
        if i2c_driver is None:
            self.i2c = busio.I2C(board.SCL, board.SDA)
        else:
            self.i2c = i2c_driver

    def is_connected(self):
        try:
            while not self.i2c.try_lock(): pass
            return self.address in self.i2c.scan()
        finally:
            self.i2c.unlock()

    @property
    def connected(self):
        return self.is_connected()

    def begin(self):
        try:
            while not self.i2c.try_lock(): pass
            self.i2c.readfrom(self.address, 1)  # dummy read
            return self.i2c.readfrom(self.address, 1)
        finally:
            self.i2c.unlock()

    def ready(self):
        try:
            while not self.i2c.try_lock(): pass
            status_byte = self.i2c.readfrom(self.address, 1)[0]
        finally:
            self.i2c.unlock()
        return status_byte & 0x01 and status_byte != 0xFF

    def busy(self):
        try:
            while not self.i2c.try_lock(): pass
            status_byte = self.i2c.readfrom(self.address, 1)[0]
        finally:
            self.i2c.unlock()
        return status_byte & (0x02 | 0x04 | 0x08) != 0

    def enable(self):
        try:
            while not self.i2c.try_lock(): pass
            self.i2c.writeto(self.address, bytes([0x70, 0x01]))
        finally:
            self.i2c.unlock()

    def disable(self):
        try:
            while not self.i2c.try_lock(): pass
            self.i2c.writeto(self.address, bytes([0x70, 0x00]))
        finally:
            self.i2c.unlock()

    def set_drive(self, motor_num, direction, level):
        level = round((level + 1 - direction) / 2)
        drive_value = 0
        if motor_num < 34:
            drive_value = (level * direction) + (level * (direction - 1))
            drive_value += 128
            try:
                while not self.i2c.try_lock(): pass
                self.i2c.writeto(self.address, bytes([0x20 + motor_num, drive_value]))
            finally:
                self.i2c.unlock()

    def inversion_mode(self, motor_num, polarity):
        reg_temp = 0
        if motor_num < 2:
            try:
                while not self.i2c.try_lock(): pass
                self.i2c.writeto(self.address, bytes([0x12 + motor_num, polarity & 0x01]))
            finally:
                self.i2c.unlock()
        else:
            if motor_num < 10:
                reg_temp = 0x50
                motor_num -= 2
            elif motor_num < 18:
                reg_temp = 0x51
                motor_num -= 10
            elif motor_num < 26:
                reg_temp = 0x52
                motor_num -= 18
            elif motor_num < 34:
                reg_temp = 0x53
                motor_num -= 26
            else:
                return
            try:
                while not self.i2c.try_lock(): pass
                data = self.i2c.readfrom(self.address, 1)[0] & (~(1 << motor_num) & 0xFF)
                self.i2c.writeto(self.address, bytes([reg_temp, data | ((polarity & 0x01) << motor_num)]))
            finally:
                self.i2c.unlock()

    def bridging_mode(self, driver_num, bridged):
        reg_temp = 0
        if driver_num < 1:
            try:
                while not self.i2c.try_lock(): pass
                self.i2c.writeto(self.address, bytes([0x14, bridged & 0x01]))
            finally:
                self.i2c.unlock()
        else:
            if driver_num < 9:
                reg_temp = 0x54
                driver_num -= 1
            elif driver_num < 17:
                reg_temp = 0x55
                driver_num -= 9
            else:
                return
            try:
                while not self.i2c.try_lock(): pass
                data = self.i2c.readfrom(self.address, 1)[0] & (~(1 << driver_num) & 0xFF)
                self.i2c.writeto(self.address, bytes([reg_temp, data | ((bridged & 0x01) << driver_num)]))
            finally:
                self.i2c.unlock()

    def get_diagnostics(self):
        my_diag = SCMDDiagnostics()
        try:
            while not self.i2c.try_lock(): pass
            my_diag.U_I2C_RD_ERR = self.i2c.readfrom(self.address, 1)[0]
            my_diag.U_I2C_WR_ERR = self.i2c.readfrom(self.address, 1)[0]
            my_diag.U_BUF_DUMPED = self.i2c.readfrom(self.address, 1)[0]
            my_diag.E_I2C_RD_ERR = self.i2c.readfrom(self.address, 1)[0]
            my_diag.E_I2C_WR_ERR = self.i2c.readfrom(self.address, 1)[0]
            my_diag.LOOP_TIME = self.i2c.readfrom(self.address, 1)[0]
            my_diag.SLV_POLL_CNT = self.i2c.readfrom(self.address, 1)[0]
            top_addr = self.i2c.readfrom(self.address, 1)[0]
            if top_addr >= 0x50 and top_addr < 0x60:
                my_diag.numberOfSlaves = top_addr - 0x50 + 1
            my_diag.MST_E_ERR = self.i2c.readfrom(self.address, 1)[0]
            my_diag.MST_E_STATUS = self.i2c.readfrom(self.address, 1)[0]
            my_diag.FSAFE_FAULTS = self.i2c.readfrom(self.address, 1)[0]
            my_diag.REG_OOR_CNT = self.i2c.readfrom(self.address, 1)[0]
            my_diag.REG_RO_WRITE_CNT = self.i2c.readfrom(self.address, 1)[0]
        finally:
            self.i2c.unlock()
        return my_diag

    def get_remote_diagnostics(self, address):
        my_diag = SCMDDiagnostics()
        my_diag.numberOfSlaves = 0
        my_diag.U_I2C_RD_ERR = 0

    # ****************************************************************************#
    #
    #   Fault Safe Section
    #
    # ****************************************************************************#

    # fault_safe_drive( ... )
    #
    #    Disable drive to both motors
    #
    def fault_safe_drive(self):
        """
            Disable drive to both motors

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_DRIVE_KILL)
        finally:
            self.i2c.unlock()

    # fault_safe_restart( ... )
    #
    #     Restart motor drives
    #
    def fault_safe_restart(self):
        """
            Restart motor drives

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_RESTART_MASK)
        finally:
            self.i2c.unlock()

    # fault_safe_reboot( ... )
    #
    #     Reboot slave controllers
    #
    def fault_safe_reboot(self):
        """
            Reboot slave controllers

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_REBOOT)
        finally:
            self.i2c.unlock()

    # fault_safe_re_enum( ... )
    #
    #     Re-enumerate slave controllers
    #
    def fault_safe_re_enum(self):
        """
            Re-enumerate slave controllers

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_RE_ENUM)
        finally:
            self.i2c.unlock()

    # fault_safe_cycle_user( ... )
    #
    #     Cycle the user voltage on all slaves
    #
    def fault_safe_cycle_user(self):
        """
            Cycle the user voltage on all slaves

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_CYCLE_USER)
        finally:
            self.i2c.unlock()

    # fault_safe_cycle_exp( ... )
    #
    #     Cycle the expansion voltage on all slaves
    #
    def fault_safe_cycle_exp(self):
        """
            Cycle the expansion voltage on all slaves

            :return: No return value

        """
        try:
            while not self.i2c.try_lock(): pass
            self._i2c.writeByte(self.address, self.SCMD_FSAFE_CTRL, self.SCMD_FSAFE_CYCLE_EXP)
        finally:
            self.i2c.unlock()

    # ****************************************************************************#
    #
    #   Misc Section
    #
    # ****************************************************************************#

    # getPage( ... )
    #
    #     Get the I2C page the SCMD is currently using
    #
    def get_page(self):
        """
            Get the I2C page the SCMD is currently using

            :return: Returns the page number
            :rtype: integer

        """

        # first read
        try:
            while not self.i2c.try_lock(): pass
            data = self._i2c.readByte(self.address, self.SCMD_PAGE_SELECT)
        finally:
            self.i2c.unlock()

        # second read
        try:
            while not self.i2c.try_lock(): pass
            return self._i2c.readByte(self.address, self.SCMD_PAGE_SELECT)
        finally:
            self.i2c.unlock()

    # select_page( ... )
    #
    #     Set the I2C page the SCMD is currently using
    #
    #   page -- Page number 0 to 3
    def select_page(self, page):
        """
            Set the I2C page the SCMD is currently using

            :param page: Page number 0 to 3

            :return: No return value

        """

        # Read the status reg
        current_page = self.get_page()

        if current_page != page:
            try:
                while not self.i2c.try_lock(): pass
                self._i2c.writeByte(self.address, self.SCMD_PAGE_SELECT, page)
            finally:
                self.i2c.unlock()
            return True
        else:
            return False

    # get_user_voltage( ... )
    #
    #     Get the user voltage for a motor controller
    #
    #   controllerNum -- Controller number from 0 to 16
    def get_user_voltage(self, controllerNum):
        """
            Get the user voltage for a motor controller

            :param controllerNum: Controller number from 0 to 16

            :return: Returns the user voltage value
            :rtype: integer

        """

        if controllerNum < 1:
            try:
                while not self.i2c.try_lock(): pass
                return self._i2c.readByte(self.address, self.SCMD_U_PORT_CLKDIV_U)
            finally:
                self.i2c.unlock()

        else:
            try:
                while not self.i2c.try_lock(): pass
                return self._i2c.readByte(self.address, self.SCMD_E_PORT_CLKDIV_U)
            finally:
                self.i2c.unlock()


    # set_user_voltage( ... )
    #
    #     Set the user voltage for a motor controller
    #
    #   controllerNum -- Controller number from 0 to 16
    #   voltage -- 0 to 255 for user voltage
    def set_user_voltage(self, controllerNum, voltage):
        """
            Set the user voltage for a motor controller

            :param controllerNum: Controller number from 0 to 16
            :param voltage: 0 to 255 for user voltage

            :return: No return value

        """

        if controllerNum < 1:
            try:
                while not self.i2c.try_lock(): pass
                self._i2c.writeByte(self.address, self.SCMD_U_PORT_CLKDIV_U, voltage)
            finally:
                self.i2c.unlock()

        else:
            try:
                while not self.i2c.try_lock(): pass
                self._i2c.writeByte(self.address, self.SCMD_E_PORT_CLKDIV_U, voltage)
            finally:
                self.i2c.unlock()

    # ****************************************************************************#
    #
    #   Explictly disable some functions to help with development
    #
    # ****************************************************************************#

    # do_not_use
    #
    #     Do not use these functions as they are for internal testing only
    #
    def do_not_use(self):
        """
            Do not use these functions as they are for internal testing only

            :return: No return value

        """

        pass

# lines 353
