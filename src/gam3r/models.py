import game_objs as objs

def cube():
    cube_model = []

    triangle1 = [
        [1,1,-5],
        [1,-1,-5],
        [-1,-1,-5]
    ]

    triangle2 = [
        [1,1,-5],
        [-1,1,-5],
        [-1,-1,-5]
    ]

    triangle3 = [
        [1,1,-7],
        [-1,1,-7],
        [-1,-1,-7]
    ]

    triangle4 = [
        [1,1,-7],
        [1,-1,-7],
        [-1,-1,-7]
    ]

    triangle5 = [
        [-1,1,-5],
        [-1,-1,-5],
        [-1,1,-7]
    ]

    triangle6 = [
        [-1,-1,-5],
        [-1,-1,-7],
        [-1,1,-7]
    ]

    triangle7 = [
        [1,1,-5],
        [1,-1,-5],
        [1,1,-7]
    ]

    triangle8 = [
        [1,-1,-5],
        [1,-1,-7],
        [1,1,-7]
    ]

    triangle9 = [
        [-1,1,-5],
        [1,1,-5],
        [1,1,-7]
    ]

    triangle10 = [
        [-1,1,-5],
        [-1,1,-7],
        [1,1,-7]
    ]

    triangle11 = [
        [-1,-1,-5],
        [1,-1,-5],
        [1,-1,-7]
    ]

    triangle12 = [
        [-1,-1,-5],
        [-1,-1,-7],
        [1,-1,-7]
    ]

    cube_model.append(triangle1)
    cube_model.append(triangle2)
    cube_model.append(triangle3)
    cube_model.append(triangle4)
    cube_model.append(triangle5)
    cube_model.append(triangle6)
    cube_model.append(triangle7)
    cube_model.append(triangle8)
    cube_model.append(triangle9)
    cube_model.append(triangle10)
    cube_model.append(triangle11)
    cube_model.append(triangle12)

    return cube_model

def octahedron():
    triangle2 = objs.Triangle([
            (0,0,-2),
            (-1,1,0),
            (-1,-1,0)])
    triangle3 = objs.Triangle([
        (-1,-1,0),
        (1,-1,0),
        (0,0,-2)])
    triangle4 = objs.Triangle([
        (0,0,-2),
        (1,1,0),
        (1,-1,0)])
    triangle5 = objs.Triangle([
        (0,0,2),
        (-1,1,0),
        (-1,-1,0)])
    triangle6 = objs.Triangle([
        (-1,-1,0),
        (1,-1,0),
        (0,0,2)])
    triangle7 = objs.Triangle([
        (0,0,2),
        (1,1,0),
        (1,-1,0)])
    triangle8 = objs.Triangle([
        (0,0,2),
        (1,1,0),
        (-1,1,0)])
    
    triangle_list = []
    triangle_list.append(triangle2)
    triangle_list.append(triangle3)
    triangle_list.append(triangle4)
    triangle_list.append(triangle5)
    triangle_list.append(triangle6)
    triangle_list.append(triangle7)
    triangle_list.append(triangle8)

    return triangle_list
    

    





