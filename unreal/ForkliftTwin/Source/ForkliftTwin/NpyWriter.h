// Forklift digital twin - minimal .npy (v1.0) writer so Python can np.load() sensor data directly.
#pragma once

#include "CoreMinimal.h"
#include "Misc/FileHelper.h"

namespace ForkliftNpy
{
	/** Little-endian header for a C-ordered 2D array. `Descr` is a numpy dtype string such as "<f4" or "<f2". */
	inline void AppendHeader(TArray<uint8>& Bytes, const TCHAR* Descr, int32 Rows, int32 Cols)
	{
		FString Dict = FString::Printf(TEXT("{'descr': '%s', 'fortran_order': False, 'shape': (%d, %d), }"), Descr, Rows, Cols);
		// Magic(6) + version(2) + header length(2) + header must total a multiple of 64, header ending in '\n'.
		const int32 Unpadded = 10 + Dict.Len() + 1;
		const int32 Pad = (64 - Unpadded % 64) % 64;
		Dict += FString::ChrN(Pad, TEXT(' ')) + TEXT("\n");

		const uint8 Magic[] = { 0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0 };
		Bytes.Append(Magic, UE_ARRAY_COUNT(Magic));
		const uint16 HeaderLen = static_cast<uint16>(Dict.Len());
		Bytes.Add(HeaderLen & 0xFF);
		Bytes.Add(HeaderLen >> 8);
		const FTCHARToUTF8 Ascii(*Dict);
		Bytes.Append(reinterpret_cast<const uint8*>(Ascii.Get()), Ascii.Length());
	}

	/** float32 matrix of Rows x Cols. */
	inline bool WriteF32(const FString& Path, const float* Data, int32 Rows, int32 Cols)
	{
		TArray<uint8> Bytes;
		AppendHeader(Bytes, TEXT("<f4"), Rows, Cols);
		Bytes.Append(reinterpret_cast<const uint8*>(Data), static_cast<int64>(Rows) * Cols * sizeof(float));
		return FFileHelper::SaveArrayToFile(Bytes, *Path);
	}

	/** Same data stored as float16 (half the size; ~0.25 cm resolution at 5 m for depth in cm). */
	inline bool WriteF16(const FString& Path, const float* Data, int32 Rows, int32 Cols)
	{
		TArray<uint8> Bytes;
		AppendHeader(Bytes, TEXT("<f2"), Rows, Cols);
		const int64 Count = static_cast<int64>(Rows) * Cols;
		const int32 Start = Bytes.Num();
		Bytes.AddUninitialized(Count * sizeof(uint16));
		uint16* Out = reinterpret_cast<uint16*>(Bytes.GetData() + Start);
		for (int64 i = 0; i < Count; ++i)
		{
			const FFloat16 Half(Data[i]);
			Out[i] = Half.Encoded;
		}
		return FFileHelper::SaveArrayToFile(Bytes, *Path);
	}
}
