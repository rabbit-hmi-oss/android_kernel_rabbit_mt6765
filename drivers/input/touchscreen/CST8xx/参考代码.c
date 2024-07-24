int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
	int chip_ic_checkcode,chip_ic_type,chip_ic_project_id,chip_ic_fw_version,chip_ic_checksum;
	
	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
	
	mdelay(10);

	buf[0] = 0xD1;
	buf[1] = 0xFC;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;

	//0xCACA0000
	chip_ic_checkcode = buf[3];
	chip_ic_checkcode <<= 8;
	chip_ic_checkcode |= buf[2];
	chip_ic_checkcode <<= 8;
    chip_ic_checkcode |= buf[1];
	chip_ic_checkcode <<= 8;
	chip_ic_checkcode |= buf[0];
	

	mdelay(10);

	buf[0] = 0xD2;
	buf[1] = 0x04;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;
	chip_ic_type = buf[3];
	chip_ic_type <<= 8;
	chip_ic_type |= buf[2];

	
	chip_ic_project_id = buf[1];
	>chip_ic_project_id <<= 8;
	chip_ic_project_id |= buf[0];


	mdelay(2);
	
	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;	

	chip_ic_fw_version = buf[3];
	chip_ic_fw_version <<= 8;
	chip_ic_fw_version |= buf[2];
	chip_ic_fw_version <<= 8;
	chip_ic_fw_version |= buf[1];
	chip_ic_fw_version <<= 8;
	chip_ic_fw_version |= buf[0];

	chip_ic_checksum = buf[7];
	chip_ic_checksum <<= 8;
	chip_ic_checksum |= buf[6];
	chip_ic_checksum <<= 8;
	chip_ic_checksum |= buf[5];
	chip_ic_checksum <<= 8;
	chip_ic_checksum |= buf[4];	

    buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);
	
	return 0;
}

