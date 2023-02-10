import gps

# Create a session to connect to the GPS daemon
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

# Continuously loop and retrieve GPS data
while True:
    try:
        report = session.next()
        # Wait for a valid GPS fix
        if report['class'] == 'TPV':
            if hasattr(report, 'lat'):
                print("Latitude: " + str(report.lat))
            if hasattr(report, 'lon'):
                print("Longitude: " + str(report.lon))
    except KeyError:
        pass
    except KeyboardInterrupt:
        quit()
    except StopIteration:
        session = None
        print("GPSD has terminated")