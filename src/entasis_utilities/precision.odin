package entasis_utilities

Precision_Mode :: enum u8
{
	Float_Coordinates,
	Double_Coordinates,
}

PRECISION_NAME :: #config(ENTASIS_PRECISION, "float32");
#assert(PRECISION_NAME == "float32" || PRECISION_NAME == "mixed64", "ENTASIS_PRECISION must be float32 or mixed64");

when PRECISION_NAME == "mixed64"
{
	PRECISION_MODE :: Precision_Mode.Double_Coordinates;
}
else
{
	PRECISION_MODE :: Precision_Mode.Float_Coordinates;
}
