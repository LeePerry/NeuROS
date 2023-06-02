from nodes.acknowledge import AckServer, AckClient

class SynchronisationServer:

    @classmethod
    def for_connection(cls, node, connection):
        synchronisation = connection.get_synchronisation()
        if synchronisation == "null":
            return cls(node, connection)
        if synchronisation == "sequential":
            return Sequential(node, connection)
        if synchronisation == "parallel":
            return Parallel(node, connection)
        raise Exception(f"Invalid synchronisation {synchronisation}!")

    def __init__(self, node, connection):
        self._first_send = True
        self._registration = AckServer(
            node,
            f"{connection.get_sender()}/{connection.get_name()}/register",
            connection.get_receivers())

    def pre_send(self):
        if self._first_send:
            self._registration.wait_for_all()
            self._first_send = False
        else:
            self._pre_impl()

    def post_send(self):
        self._post_impl()

    def _pre_impl(self):
        pass

    def _post_impl(self):
        pass

class Sequential(SynchronisationServer):

    def __init__(self, node, connection):
        super().__init__(node, connection)
        self._acknowledgement = AckServer(
            node,
            f"{connection.get_sender()}/{connection.get_name()}/acknowledge",
            connection.get_receivers())

    def _post_impl(self):
        self._acknowledgement.wait_for_all()
        self._acknowledgement.reset()

class Parallel(SynchronisationServer):

    def __init__(self, node, connection):
        super().__init__(node, connection)
        self._acknowledgement = AckServer(
            node,
            f"{connection.get_sender()}/{connection.get_name()}/acknowledge",
            connection.get_receivers(),
            connection.get_max_permitted_no_ack())

    def _pre_impl(self):
        self._acknowledgement.wait_for_all()
        self._acknowledgement.reset()

class SynchronisationClient:

    @classmethod
    def for_connection(cls, node, connection):
        return cls(node, connection)

    def __init__(self, node, connection):
        self._registration = AckClient(
            node,
            f"{connection.get_sender()}/{connection.get_name()}/register")
        self._acknowledgement = AckClient(
            node,
            f"{connection.get_sender()}/{connection.get_name()}/acknowledge")
        self._registration.start_timer(0.2)

    def stop_registration(self):
        self._registration.stop_timer()

    def acknowledge_packet(self):
        self._acknowledgement.send()
