import qrcode

for i in range(9):
	qr = qrcode.QRCode(
	    version=1,
	    error_correction=qrcode.constants.ERROR_CORRECT_H,
	    box_size=200,
	    border=10,
	)
	qr.add_data(str(i))
	qr.make(fit=True)

	img = qr.make_image(fill_color="black", back_color="white")

	img.save(str(i)+'.png')
