#include <tft_driver.hpp>
#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>
#include <gfx_bitmap.hpp>
namespace arduino {
    template<uint16_t Width,
            uint16_t Height,
            typename Bus,
            uint8_t Rotation = 0,
            size_t BitDepth = 1,
            uint8_t Address = 0x3C,
            bool Vdc3_3=true,
            uint32_t WriteSpeedPercent=400,
            int8_t PinDC=-1,
            int8_t PinRst=-1,
            bool ResetBeforeInit=false>
    struct ssd1306 final {
        
        constexpr static const uint8_t rotation = Rotation &3;
        constexpr static const size_t bit_depth = BitDepth;
        constexpr static const bool dithered = bit_depth!=1;
        constexpr static const uint8_t address = Address;
        constexpr static const bool vdc_3_3 = Vdc3_3;
        constexpr static const float write_speed_multiplier = (WriteSpeedPercent/100.0);
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const bool reset_before_init = ResetBeforeInit;
private:
        constexpr static const uint16_t width=Width;
        constexpr static const uint16_t height=Height;
        
        using bus = Bus;
        using driver = tft_driver<PinDC,PinRst,-1,Bus,-1,0x3C,0x00,0x40>;
        using frame_buffer_type = gfx::large_bitmap<gfx::gsc_pixel<bit_depth>>;
        unsigned int m_initialized;
        unsigned int m_suspend_count;
        uint8_t m_contrast;
        frame_buffer_type m_frame_buffer;
        gfx::rect16 m_suspend_bounds;
        bool m_dithering;
        inline void write_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(is_data) {
                driver::send_data(data,size);
            } else {
                driver::send_command(data,size);
            }
        }
        inline void write_pgm_bytes(const uint8_t* data,size_t size,bool is_data) {
            if(is_data) {
                driver::send_data_pgm(data,size);
            } else {
                driver::send_command_pgm(data,size);
            }
        }
        static void expand_rect(gfx::rect16& dst, const gfx::rect16& src) {
            if(dst.x1==uint16_t(-1) || dst.y1==uint16_t(-1) || dst.x2==uint16_t(-1) || dst.y2==uint16_t(-1)) {
                dst=src;
                return;
            }
            if(src.x1<dst.x1) {
                dst.x1 = src.x1;
            }
            if(src.y1<dst.y1) {
                dst.y1 = src.y1;
            }
            if(src.x2>dst.x2) {
                dst.x2 = src.x2;
            }
            if(src.y2>dst.y2) {
                dst.y2 = src.y2;
            }
        }

        void update_display() {
            if(m_suspend_count) {
                return;
            }
            
            gfx::rect16 rr = m_suspend_bounds;
            m_suspend_bounds = gfx::rect16(-1,-1,-1,-1);
            translate(rr);
            if(!rr.intersects(gfx::rect16(0,0,width-1,height-1))) {
                return;
            }
            //print_source(m_frame_buffer);
            pixel_type cpx;
            rr.y1&=0xF8;
            rr.y2|=0x07;
            uint8_t dlist1[] = {
                    0x22,
                    uint8_t(rr.y1/8),                   // Page start address
                    uint8_t(0xFF),                   // Page end (not really, but works here)
                    0x21, uint8_t(rr.x1)};// Column start address
            write_bytes(dlist1, sizeof(dlist1),false);
            uint8_t col = rr.x2;
            write_bytes(&col,1,false); // Column end address
            
            if(!dithered) {   
                for(int y = rr.y1;y<=rr.y2;y+=8) {
                    for(int x = rr.x1;x<=rr.x2;++x) {
                        uint8_t b = 0;
                        for(int yy = 0;yy<8;++yy) {
                            uint16_t tmp_x=x,tmp_y=yy+y;
                            translate(tmp_x,tmp_y);
                            m_frame_buffer.point({tmp_x,tmp_y},&cpx);
                            if(cpx.native_value) {
                                b|=(1<<(yy));
                            }
                        }
                        write_bytes(&b,1,true);
                    }
                }
            } else {
                if(m_dithering) {
                    for(int y = rr.y1;y<=rr.y2;y+=8) {
                        for(int x = rr.x1;x<=rr.x2;++x) {
                            int col = x&15;
                            uint8_t b = 0;
                            for(int yy = 0;yy<8;++yy) {
                                int yyy = yy+y;
                                int row=yyy&15;
                                uint16_t tmp_x=x,tmp_y=yyy;
                                translate(tmp_x,tmp_y);
                                m_frame_buffer.point({tmp_x,tmp_y},&cpx);
                                
                                b|=(1<<(yy))*(255.0*cpx.template channelr<gfx::channel_name::L>()>gfx::helpers::dither::bayer_16[col+row*16]);
                                
                                
                            }
                            write_bytes(&b,1,true);
                        }
                    }
                } else {
                    for(int y = rr.y1;y<=rr.y2;y+=8) {
                        for(int x = rr.x1;x<=rr.x2;++x) {
                            uint8_t b = 0;
                            for(int yy = 0;yy<8;++yy) {
                                uint16_t tmp_x=x,tmp_y=yy+y;
                                translate(tmp_x,tmp_y);
                                m_frame_buffer.point({tmp_x,tmp_y},&cpx);
                                gfx::gsc_pixel<1> npx;
                                gfx::convert(cpx,&npx);
                                if(npx.native_value) {
                                    b|=(1<<(yy));
                                }
                            }
                            write_bytes(&b,1,true);
                        }
                    }   
                }
            }
        }
        
        inline static void translate(uint16_t& x,uint16_t& y,bool flip=true) {
            uint16_t tmp;
            if(rotation&1) {
                tmp=y;
                y=x;
                x=tmp;                 
            };
        }
        inline static void translate(gfx::point16& point) {
            translate(point.x,point.y);
        }
        inline static void translate(gfx::rect16& rect) {
            translate(rect.x1,rect.y1,false);
            translate(rect.x2,rect.y2,false);
            //rect.normalize_inplace();
        }
public:
        ssd1306(void*(allocator)(size_t)=::malloc,void(deallocator)(void*)=::free) : 
                    m_initialized(false),
                    m_suspend_count(0),
                    m_frame_buffer(dimensions(),1,nullptr,allocator,deallocator),
                    m_suspend_bounds(-1,-1,-1,-1),
                    m_dithering(dithered) {
            
        }
        inline bool initialized() const {
            return m_initialized;
        }
        void reset() {
            if(pin_rst>=0) {
                digitalWrite(pin_rst,HIGH);
                delay(1);
                digitalWrite(pin_rst,LOW);
                delay(10);
                digitalWrite(pin_rst,HIGH);
            }
        }
        gfx::gfx_result initialize() {
            if(!m_initialized) {
                if(!m_frame_buffer.initialized()) {
                    return gfx::gfx_result::out_of_memory;
                }
                if(!driver::initialize()) {
                    return gfx::gfx_result::device_error;
                }
                bus::set_speed_multiplier(write_speed_multiplier);
                if(reset_before_init) {
                    reset();
                }
                bus::begin_initialization();
                bus::begin_write();
                uint8_t cmd;
                // Init sequence
                static const uint8_t init1[] PROGMEM = {0xAE,
                                                        0xD5,
                                                        0x80, // the suggested ratio 0x80
                                                        0xA8};
                write_pgm_bytes(init1, sizeof(init1),false);

                cmd=height-1;
                write_bytes(&cmd,1,false);
                
                static const uint8_t init2[] PROGMEM = {0xD3,
                                                        0x00,                      // no offset
                                                        0x40 | 0x00, // line #0
                                                        0x8D};
                write_pgm_bytes(init2, sizeof(init2),false);
                
                cmd=!vdc_3_3 ? 0x10 : 0x14;
                write_bytes(&cmd,1,false);

                static const uint8_t init3[] PROGMEM = { 0x20,
                                                        0x00, // 0x0 act like ks0108
                                                        0xA0 | 0x1,
                                                        0xC8};
                write_pgm_bytes(init3, sizeof(init3),false);
                uint8_t com_pins = 0x02;
                m_contrast = 0x8F;
                if ((width == 128) && (height == 32)) {
                    com_pins = 0x02;
                    m_contrast = 0x8F;
                } else if ((width == 128) && (height == 64)) {
                    com_pins = 0x12;
                    m_contrast = !vdc_3_3 ? 0x9F:0xCF;
                } else if ((width == 96) && (height == 16)) {
                    com_pins = 0x2; // ada x12
                    m_contrast = !vdc_3_3 ? 0x10:0xAF;
                } else
                    return gfx::gfx_result::invalid_argument;
                cmd=0xDA;
                write_bytes(&cmd,1,false);
                write_bytes(&com_pins,1,false);
                cmd=0x81;
                write_bytes(&cmd,1,false);
                write_bytes(&m_contrast,1,false);
                cmd=0xD9;
                write_bytes(&cmd,1,false);
                cmd=!vdc_3_3 ? 0x22:0xF1;
                write_bytes(&cmd,1,false);
                static const uint8_t init5[] PROGMEM = {
                    0xDB,
                    0x40,
                    0xA4,
                    0xA6,
                    0x2E,
                    0xAF}; // Main screen turn on
                write_pgm_bytes(init5, sizeof(init5),false);
                bus::end_write();
                bus::end_initialization();
                m_suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
                m_suspend_count = 0;
                m_initialized = true;
            }
            return gfx::gfx_result::success;
        }
        inline bool dithering() const {
            return m_dithering && dithered;
        }
        inline void dithering(bool value) {
            m_dithering = value;
        }
        // GFX Bindings
        using type = ssd1306;
        using pixel_type = gfx::gsc_pixel<bit_depth>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        constexpr inline gfx::size16 dimensions() const {return rotation&1?gfx::size16(height,width):gfx::size16(width,height);}
        constexpr inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        // gets a point 
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            if(!m_initialized) {
                return gfx::gfx_result::invalid_state;
            }
            if(rotation==1) {
                location.x = m_frame_buffer.dimensions().width-1-location.x;
            } else if(rotation==2) {
                location.x = m_frame_buffer.dimensions().width-1-location.x;
                location.y = m_frame_buffer.dimensions().height-1-location.y;
            } else if(rotation==3) {
                location.y = m_frame_buffer.dimensions().height-1-location.y;
            }
            return m_frame_buffer.point(location,out_color);
       }
        // sets a point to the specified pixel
        inline gfx::gfx_result point(gfx::point16 location,pixel_type color) {
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            if(!bounds().intersects(location)) {
                return gfx::gfx_result::success;
            }
            if(rotation==1) {
                location.x = m_frame_buffer.dimensions().width-1-location.x;
            } else if(rotation==2) {
                location.x = m_frame_buffer.dimensions().width-1-location.x;
                location.y = m_frame_buffer.dimensions().height-1-location.y;
            } else if(rotation==3) {
                location.y = m_frame_buffer.dimensions().height-1-location.y;
            }
            expand_rect(m_suspend_bounds,{location.x,location.y,location.x,location.y});
            m_frame_buffer.point(location,color);
            update_display();
            return gfx::gfx_result::success;    
        }
        inline gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            gfx::rect16 rect = bounds.crop(this->bounds()).normalize();
            if(!this->bounds().intersects(rect)) {
                return gfx::gfx_result::success;
            }
            if(rotation==1) {
                rect.x1 = m_frame_buffer.dimensions().width-1-rect.x1;
                rect.x2 = m_frame_buffer.dimensions().width-1-rect.x2;
                rect.normalize_inplace();
            } else if(rotation==2) {
                rect.x1 = m_frame_buffer.dimensions().width-1-rect.x1;
                rect.x2 = m_frame_buffer.dimensions().width-1-rect.x2;
                rect.y1 = m_frame_buffer.dimensions().height-1-rect.y1;
                rect.y2 = m_frame_buffer.dimensions().height-1-rect.y2;
                rect.normalize_inplace();
            } else if(rotation==3) {
                rect.y1 = m_frame_buffer.dimensions().height-1-rect.y1;
                rect.y2 = m_frame_buffer.dimensions().height-1-rect.y2;
                rect.normalize_inplace();
            }
            expand_rect(m_suspend_bounds,rect);
            m_frame_buffer.fill(rect,color);
            update_display();
            return gfx::gfx_result::success;
        }
        
        // clears the specified rectangle
        inline gfx::gfx_result clear(const gfx::rect16& rect) {
            pixel_type p;
            return fill(rect,p);
        }
        inline gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            if(m_suspend_count<2 || force) {
                m_suspend_count = 0;
                update_display();
                return gfx::gfx_result::success;
            }
            --m_suspend_count;
            return gfx::gfx_result::success;
        }
    };
}