from zmqRemoteApi.clients.python.zmqRemoteApi import RemoteAPIClient


if __name__ == '__main__':
    # Connect to Coppelia simulation
    client = RemoteAPIClient()
    simulation = client.getObject('sim')
    simulation.stopSimulation()

    '''
    Simulation code gets called here
    '''

    sim = simulation.startSimulation()
