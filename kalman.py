import numpy as np

I = np.identity(3)
A_t = np.identity(3)

def z(C_t, x_t, delta_t):
    return C_t*x_t + delta_t

def kalman_filter(previous_mu ,previous_sigma,current_u ,z_t, A_t, B_t, R_t, C_t, Q_t):

    #Prediction
    next_mu_bar= A_t*previous_mu + B_t*current_u
    next_sigma_bar= A_t*previous_sigma*A_t.transpose() + R_t

    #Correction
    K_t= next_sigma_bar*C_t.transpose *(C_t*next_sigma_bar*C_t.transpose()+ Q_t).inverse()
    next_mu= next_mu_bar + K_t(z_t - C_t*next_mu_bar)
    next_sigma=(I-K_t*C_t)*next_sigma_bar

    return (next_mu,next_sigma)
