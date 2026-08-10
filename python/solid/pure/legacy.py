import numpy as np

def xy2theta(x, y):
    if (x >= 0 and y >= 0): 
        theta = 180/np.pi * np.arctan(y/x)
    if (x < 0 and y >= 0): 
        theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
    if (x < 0 and y < 0): 
        theta = 180 + ((180/np.pi) * np.arctan(y/x))
    if ( x >= 0 and y < 0):
        theta = 360 - ((180/np.pi) * np.arctan((-y)/x))
    return theta

def pt2rah(point, gap_ring, gap_sector, gap_height, num_ring, num_sector, num_height, fov_d):
    x = point[0]
    y = point[1]
    z = point[2]
    
    if(x == 0.0):
        x = 0.001  
    if(y == 0.0):
        y = 0.001 

    theta   = xy2theta(x, y) 
    faraway = np.sqrt(x*x + y*y) 
    phi     = np.rad2deg(np.arctan2(z, np.sqrt(x**2 + y**2))) - fov_d

    idx_ring   = np.divmod(faraway, gap_ring)[0]      
    idx_sector = np.divmod(theta, gap_sector)[0]   
    idx_height = np.divmod(phi, gap_height)[0]
    
    if(idx_ring >= num_ring):
        idx_ring = num_ring-1

    if(idx_height >= num_height):
        idx_height = num_height-1

    return int(idx_ring), int(idx_sector), int(idx_height)

## =========================================================================================
##                                            SOLiD
## =========================================================================================
def ptcloud2solid(ptcloud, fov_u, fov_d, num_sector, num_ring, num_height, max_length):
    num_points = ptcloud.shape[0]               
    
    gap_ring = max_length/num_ring            
    gap_sector = 360/num_sector              
    gap_height = ((fov_u-fov_d))/num_height              

    rh_counter = np.zeros([num_ring, num_height])             
    sh_counter = np.zeros([num_sector, num_height])   
    for pt_idx in range(num_points): 
        point = ptcloud[pt_idx, :]
        idx_ring, idx_sector, idx_height = pt2rah(point, gap_ring, gap_sector, gap_height, num_ring, num_sector, num_height, fov_d) 
        try :
            rh_counter[idx_ring, idx_height] = rh_counter[idx_ring, idx_height] + 1     
            sh_counter[idx_sector, idx_height] = sh_counter[idx_sector, idx_height] + 1  
        except:
            pass
            
    ring_matrix = rh_counter    
    sector_matrix = sh_counter
    number_vector = np.sum(ring_matrix, axis=0)
    min_val = number_vector.min()
    max_val = number_vector.max()
    number_vector = (number_vector - min_val) / (max_val - min_val)
        
    r_solid = ring_matrix.dot(number_vector)
    a_solid = sector_matrix.dot(number_vector)
            
    return r_solid, a_solid

def get_descriptor(scan, fov_u, fov_d, num_height, max_length):
    # divide range
    num_ring = 40
    # divide angle
    num_sector = 60

    r_solid, a_solid = ptcloud2solid(scan, fov_u, fov_d, num_sector, num_ring, num_height, max_length)

    return r_solid, a_solid