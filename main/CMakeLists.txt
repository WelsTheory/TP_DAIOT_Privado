idf_component_register(SRCS "aws-iot.c" "TP_DAIOT.c" "DHT22.c" 
                    INCLUDE_DIRS ".")
target_add_binary_data(${COMPONENT_TARGET} "certs/aws_root_ca_pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate_pem_crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private_pem_key" TEXT)