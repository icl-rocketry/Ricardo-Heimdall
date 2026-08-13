#pragma once

#include <SPI.h>
#include <SdFat.h>

#include <libriccore/storage/storebase.h>
#include <libriccore/threading/riccorethread.h>
#include <libriccore/riccoretypes.h>
#include <libriccore/riccorelogging.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"

#include "Storage/sdfat_file.h"



class SdFat_Store : public StoreBase
{
    public:
        SdFat_Store(SPIClass &spi,const uint8_t cs,const uint32_t frequency,RicCoreThread::Lock_t &spiBusLock,bool dedicatedSPI,Types::CoreTypes::SystemStatus_t* systemstatus);
        SdFat_Store(SPIClass &spi,const uint8_t cs, const uint32_t frequency,bool dedicatedSPI,Types::CoreTypes::SystemStatus_t* systemstatus);

        void setup();
        uint8_t getError(){
            return filesys.sdErrorCode();
        }

        ~SdFat_Store(){};
    protected:
         std::unique_ptr<WrappedFile> _open(std::string_view path, store_fd fileDesc, FILE_MODE mode, size_t maxQueueSize) override;
         bool _ls(std::string_view path, std::vector<directory_element_t> &directory_structure) override;
         bool _mkdir(std::string_view path) override;
         bool _remove(std::string_view path) override;


    private:

        SPIClass &_spi;

        const SdSpiConfig _config;

        RicCoreThread::Lock_t dummyLock;

        Types::CoreTypes::SystemStatus_t* _systemstatus;

        SdFs filesys;



};