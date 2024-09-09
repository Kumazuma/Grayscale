#include "pch.h"

enum {
    ID_PICKER = wxID_HIGHEST + 1,
    ID_SLIDER,
    ID_PAINT_PANEL,
    ID_SPIN_CTRL
};

class GrayscaleApp : public wxApp
{
public:
    bool OnInit() override;

protected:
    void UpdatedValue();

private:
    wxWeakRef<wxFrame> m_frame;
    wxSpinCtrl* m_spinCtrl;
    wxFilePickerCtrl* m_filePickerCtrl;
    wxSlider* m_sliderCtrl;
    wxPanel* m_paintPanel;
    
    int m_currentValue;
    wxImage m_originImage;
    wxBitmap m_convertedBitmap;
    wxImage m_convertedImage;
};

wxIMPLEMENT_APP(GrayscaleApp);

bool GrayscaleApp::OnInit()
{
    if (!wxApp::OnInit())
    {
        return false;
    }

    wxInitAllImageHandlers();
    m_currentValue = 255;
    auto* frame = new wxFrame(nullptr, wxID_ANY, wxS("Grayscale"));
    auto* picker = new wxFilePickerCtrl(frame, ID_PICKER, {}, wxS("Images files"), wxS("image files|*.jpeg;*.png;*.jpg"));
    auto* slider = new wxSlider(frame, ID_SLIDER, m_currentValue, 1, 255);
    auto* spinCtrl = new wxSpinCtrl(frame, ID_SPIN_CTRL, wxS("255"));
    spinCtrl->SetMin(1);
    spinCtrl->SetMax(255);
    spinCtrl->SetValue(m_currentValue);
    auto* paintPanel = new wxPanel(frame, ID_PAINT_PANEL);
    auto* boxSizer = new wxBoxSizer(wxVERTICAL);
    auto* boxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    boxSizer->Add(picker, 0, wxEXPAND | wxALL, 5);
    boxSizer2->Add(slider, 1, wxEXPAND | wxALL, 5);
    boxSizer2->Add(spinCtrl, 0, wxEXPAND | wxALL, 5);
    boxSizer->Add(boxSizer2, 0, wxEXPAND | wxALL, 5);
    boxSizer->Add(paintPanel, 1, wxEXPAND | wxALL, 5);
    frame->SetSizer(boxSizer);
    frame->Layout();
    frame->Show();
    paintPanel->SetBackgroundStyle(wxBG_STYLE_PAINT);
    m_frame = frame;
    m_paintPanel = paintPanel;
    m_spinCtrl = spinCtrl;
    m_sliderCtrl = slider;
    m_filePickerCtrl = picker;
    spinCtrl->Bind(wxEVT_SPINCTRL,
        [this](auto& event)
        {
            m_currentValue = event.GetSelection();
            UpdatedValue();
        });

    slider->Bind(wxEVT_SLIDER,
        [this](auto& event)
        {
            m_currentValue = event.GetSelection();
            UpdatedValue();
        });

    picker->Bind(wxEVT_FILEPICKER_CHANGED,
        [this](auto& event)
        {
            auto path = m_filePickerCtrl->GetPath();
            wxImage image;
            if (!image.LoadFile(path))
            {
                wxMessageBox(wxS("Failed to load the image file"), wxS("Grayscale"));
                return;
            }

            m_originImage = image;
            m_convertedImage = wxImage{m_originImage.GetSize()};
            UpdatedValue();
        });

    m_paintPanel->Bind(wxEVT_PAINT, [this](auto&)
        {
            if (!m_convertedBitmap.IsOk())
                return;

            wxBufferedPaintDC dc{ m_paintPanel };
            dc.Clear();
            dc.DrawBitmap(m_convertedBitmap, { 0, 0 });
        });
    return true;
}

void GrayscaleApp::UpdatedValue()
{
    m_spinCtrl->SetValue(m_currentValue);
    m_sliderCtrl->SetValue(m_currentValue);
    if (!m_originImage.IsOk())
    {
        return;
    }

    const uint8_t* const rgbArray = m_originImage.GetData();
    auto originImageSize = m_originImage.GetSize();
    const auto numOfPixel = originImageSize.x * originImageSize.y;

    // RGB에서 Brightness를 가져온다.
    const auto s = numOfPixel / 4;
    __m128 floorThreshold = _mm_set_ps1(m_currentValue);
    __m128 floorThresholdReverse = _mm_div_ps(_mm_set_ps1(1.f), floorThreshold);
    for (int i = 0; i < s; ++i)
    {
        //__m128i pixel0 = _mm_loadu_si16(rgbArray + (i * 4 + 0) * 3);
        //pixel0.m128i_u8[2] = *(rgbArray + (i * 4 + 0) * 3 + 2);

        //__m128i pixel1 = _mm_loadu_si16(rgbArray + (i * 4 + 1) * 3);
        //pixel1.m128i_u8[2] = *(rgbArray + (i * 4 + 1) * 3 + 2);

        //__m128i pixel2 = _mm_loadu_si16(rgbArray + (i * 4 + 2) * 3);
        //pixel2.m128i_u8[2] = *(rgbArray + (i * 4 + 2) * 3 + 2);

        //__m128i pixel3 = _mm_loadu_si16(rgbArray + (i * 4 + 3) * 3);
        //pixel3.m128i_u8[2] = *(rgbArray + (i * 4 + 3) * 3 + 2);

        //pixel0 = _mm_cvtepu8_epi32(pixel0);
        //pixel1 = _mm_cvtepu8_epi32(pixel1);
        //pixel2 = _mm_cvtepu8_epi32(pixel2);
        //pixel3 = _mm_cvtepu8_epi32(pixel3);

        //__m128 pixel0AsFloat = _mm_cvtepi32_ps(pixel0);
        //__m128 pixel1AsFloat = _mm_cvtepi32_ps(pixel1);
        //__m128 pixel2AsFloat = _mm_cvtepi32_ps(pixel2);
        //__m128 pixel3AsFloat = _mm_cvtepi32_ps(pixel3);

        //__m128 R0G0R1G1 = _mm_shuffle_ps(pixel0AsFloat, pixel1AsFloat, _MM_SHUFFLE(1, 0, 1, 0));
        //__m128 B0A0B1A1 = _mm_shuffle_ps(pixel0AsFloat, pixel1AsFloat, _MM_SHUFFLE(3, 2, 3, 2));
        //__m128 R2G2R3G3 = _mm_shuffle_ps(pixel2AsFloat, pixel3AsFloat, _MM_SHUFFLE(1, 0, 1, 0));
        //__m128 B2A2B3A3 = _mm_shuffle_ps(pixel2AsFloat, pixel3AsFloat, _MM_SHUFFLE(3, 2, 3, 2));

        // 4 바이트로 세 번 읽고 셔플해서 픽셀 네 개로 쪼갠다.
        // 위의 코드를 개선함
        __m128i bytes0 = _mm_loadu_si32(rgbArray + i * 4 * 3 + 0);
        __m128i bytes1 = _mm_loadu_si32(rgbArray + i * 4 * 3 + 4);
        __m128i bytes2 = _mm_loadu_si32(rgbArray + i * 4 * 3 + 8);

        bytes0 = _mm_cvtepu8_epi32(bytes0);
        bytes1 = _mm_cvtepu8_epi32(bytes1);
        bytes2 = _mm_cvtepu8_epi32(bytes2);

        // [R0, G0, B0, R1]
        __m128 components0AsFloat = _mm_cvtepi32_ps(bytes0);
        // [G1, B1, R2, G2]
        __m128 components1AsFloat = _mm_cvtepi32_ps(bytes1);
        // [B2, R3, G3, B3]
        __m128 components2AsFloat = _mm_cvtepi32_ps(bytes2);

        __m128 R0G0R1G1;
        __m128 B0A0B1A1;
        __m128 R2G2R3G3;
        __m128 B2A2B3A3;
        // [R0, G0, G1, G1]
        R0G0R1G1 = _mm_shuffle_ps(components0AsFloat, components1AsFloat, _MM_SHUFFLE(0, 0, 1, 0));

        // [R0, G0, R1, G1]
        R0G0R1G1 = _mm_insert_ps(R0G0R1G1, components0AsFloat, _MM_MK_INSERTPS_NDX(3, 2, 0));

        // [B0, B0, B1, B1]
        B0A0B1A1 = _mm_shuffle_ps(components0AsFloat, components1AsFloat, _MM_SHUFFLE(1, 1, 2, 2));

        // [R2, G2, R3, G3]
        R2G2R3G3 = _mm_shuffle_ps(components1AsFloat, components2AsFloat, _MM_SHUFFLE(2, 1, 3, 2));

        // [B2, B2, B3, B3]
        B2A2B3A3 = _mm_shuffle_ps(components2AsFloat, components2AsFloat, _MM_SHUFFLE(3, 3, 0, 0));

        // Y = 0.2126 R + 0.7152 G + 0.0722 B
        __m128 R0G0R1G1_step1 = _mm_mul_ps(R0G0R1G1, _mm_set_ps(0.7152f, 0.2126f, 0.7152f, 0.2126f));
        __m128 R2G2R3G3_step1 = _mm_mul_ps(R2G2R3G3, _mm_set_ps(0.7152f, 0.2126f, 0.7152f, 0.2126f));

        __m128 B0A0B1A1_step1 = _mm_mul_ps(B0A0B1A1, _mm_set_ps(0.f, 0.0722f, 0.f, 0.0722f));
        __m128 B2A2B3A3_step1 = _mm_mul_ps(B2A2B3A3, _mm_set_ps(0.f, 0.0722f, 0.f, 0.0722f));

        // [R0 + B0, R1 + B1, G0, G1] 
        __m128 RG0RG1 = _mm_hadd_ps(R0G0R1G1_step1, B0A0B1A1_step1); 

        // [R2 + B2, R3 + B3, G2, G3] 
        __m128 RG2RG3 = _mm_hadd_ps(R2G2R3G3_step1, B2A2B3A3_step1);

        // [R0 + B0, G0, R1 + B1, G1] 
        __m128 RB0_G0_RB1_G1 = _mm_shuffle_ps(RG0RG1, RG0RG1, _MM_SHUFFLE(3, 1, 2, 0));

        // [R0 + B0, G0, R1 + B1, G1] 
        __m128 RB2_G2_RB3_G3 = _mm_shuffle_ps(RG2RG3, RG2RG3, _MM_SHUFFLE(3, 1, 2, 0));
        
        __m128 Y0123 = _mm_hadd_ps(RB0_G0_RB1_G1, RB2_G2_RB3_G3);
        if (m_currentValue != 255)
        {
            Y0123 = _mm_mul_ps(Y0123, _mm_set_ps1(1 / 255.f));
            Y0123 = _mm_mul_ps(_mm_floor_ps(_mm_mul_ps(Y0123, floorThreshold)), floorThresholdReverse);
            Y0123 = _mm_mul_ps(Y0123, _mm_set_ps1(255.f));
        }

        __m128i Y0124AsInt = _mm_cvtps_epi32(Y0123);
        const int base = i * 4 * 3;
        uint8_t* const desc = m_convertedImage.GetData() + base;
#if defined(_M_AMD64)
        desc[0] = Y0124AsInt.m128i_u32[0];
        desc[1] = Y0124AsInt.m128i_u32[0];
        desc[2] = Y0124AsInt.m128i_u32[0];

        desc[3] = Y0124AsInt.m128i_u32[1];
        desc[4] = Y0124AsInt.m128i_u32[1];
        desc[5] = Y0124AsInt.m128i_u32[1];

        desc[6] = Y0124AsInt.m128i_u32[2];
        desc[7] = Y0124AsInt.m128i_u32[2];
        desc[8] = Y0124AsInt.m128i_u32[2];

        desc[9] = Y0124AsInt.m128i_u32[3];
        desc[10] = Y0124AsInt.m128i_u32[3];
        desc[11] = Y0124AsInt.m128i_u32[3];
#elif defined(_M_ARM64)
        desc[0] = Y0124AsInt.n128_i32[0];
        desc[1] = Y0124AsInt.n128_i32[0];
        desc[2] = Y0124AsInt.n128_i32[0];

        desc[3] = Y0124AsInt.n128_i32[1];
        desc[4] = Y0124AsInt.n128_i32[1];
        desc[5] = Y0124AsInt.n128_i32[1];

        desc[6] = Y0124AsInt.n128_i32[2];
        desc[7] = Y0124AsInt.n128_i32[2];
        desc[8] = Y0124AsInt.n128_i32[2];

        desc[9] = Y0124AsInt.n128_i32[3];
        desc[10] = Y0124AsInt.n128_i32[3];
        desc[11] = Y0124AsInt.n128_i32[3];

#endif
    }

    const auto m = numOfPixel % 4;
    for (int i = numOfPixel - m; i < numOfPixel; ++i)
    {
        float r = rgbArray[i * 3 + 0];
        float g = rgbArray[i * 3 + 1];
        float b = rgbArray[i * 3 + 2];
        // Y = 0.2126 R + 0.7152 G + 0.0722 B
        float y = r * 0.2126f + g * 0.7152f + 0.0722f * b;
        if (m_currentValue != 255)
        {
            y *= 1 / 255.f;
            y = std::floor(y * m_currentValue) / m_currentValue;
            y *= 255.f;
        }
        
        uint8_t* const desc = m_convertedImage.GetData();
        desc[i * 3 + 0] = y;
        desc[i * 3 + 1] = y;
        desc[i * 3 + 2] = y;
    }

    m_convertedBitmap = m_convertedImage;
    m_paintPanel->Refresh();
}
