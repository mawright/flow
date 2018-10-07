

class KernelVehicle(object):
    """

    """

    def __init__(self,
                 master_kernel,
                 kernel_api,
                 sim_params):
        """

        """
        self.master_kernel = master_kernel
        self.kernel_api = kernel_api
        self.sim_step = sim_params.sim_step

    def update(self, reset):
        """

        :return:
        """
        raise NotImplementedError
