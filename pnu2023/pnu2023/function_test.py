def main():
    wall_1 = [[(1,2),(3,4),(5,6)],[(7,8),(9,10)],[(11,12)]]
    for wall in wall_1:
        rhos = [rho for (rho,angle) in wall]
        print(rhos)
main()