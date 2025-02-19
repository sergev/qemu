/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * deallocation functions for schema-defined QAPI types
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Michael Roth      <mdroth@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#include "qapi/dealloc-visitor.h"
#include "qga-qapi-types.h"
#include "qga-qapi-visit.h"

const char * const GuestFsfreezeStatus_lookup[] = {
    [GUEST_FSFREEZE_STATUS_THAWED] = "thawed",
    [GUEST_FSFREEZE_STATUS_FROZEN] = "frozen",
    [GUEST_FSFREEZE_STATUS_MAX] = NULL,
};

const char * const GuestIpAddressType_lookup[] = {
    [GUEST_IP_ADDRESS_TYPE_IPV4] = "ipv4",
    [GUEST_IP_ADDRESS_TYPE_IPV6] = "ipv6",
    [GUEST_IP_ADDRESS_TYPE_MAX] = NULL,
};

const char * const GuestDiskBusType_lookup[] = {
    [GUEST_DISK_BUS_TYPE_IDE] = "ide",
    [GUEST_DISK_BUS_TYPE_FDC] = "fdc",
    [GUEST_DISK_BUS_TYPE_SCSI] = "scsi",
    [GUEST_DISK_BUS_TYPE_VIRTIO] = "virtio",
    [GUEST_DISK_BUS_TYPE_XEN] = "xen",
    [GUEST_DISK_BUS_TYPE_USB] = "usb",
    [GUEST_DISK_BUS_TYPE_UML] = "uml",
    [GUEST_DISK_BUS_TYPE_SATA] = "sata",
    [GUEST_DISK_BUS_TYPE_SD] = "sd",
    [GUEST_DISK_BUS_TYPE_MAX] = NULL,
};

const char * const GuestMemoryBlockResponseType_lookup[] = {
    [GUEST_MEMORY_BLOCK_RESPONSE_TYPE_SUCCESS] = "success",
    [GUEST_MEMORY_BLOCK_RESPONSE_TYPE_NOT_FOUND] = "not-found",
    [GUEST_MEMORY_BLOCK_RESPONSE_TYPE_OPERATION_NOT_SUPPORTED] = "operation-not-supported",
    [GUEST_MEMORY_BLOCK_RESPONSE_TYPE_OPERATION_FAILED] = "operation-failed",
    [GUEST_MEMORY_BLOCK_RESPONSE_TYPE_MAX] = NULL,
};


void qapi_free_GuestAgentCommandInfoList(GuestAgentCommandInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestAgentCommandInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestAgentCommandInfo(GuestAgentCommandInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestAgentCommandInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestAgentInfoList(GuestAgentInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestAgentInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestAgentInfo(GuestAgentInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestAgentInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileReadList(GuestFileReadList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileReadList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileRead(GuestFileRead *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileRead(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileWriteList(GuestFileWriteList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileWriteList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileWrite(GuestFileWrite *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileWrite(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileSeekList(GuestFileSeekList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileSeekList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFileSeek(GuestFileSeek *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFileSeek(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFsfreezeStatusList(GuestFsfreezeStatusList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFsfreezeStatusList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestIpAddressTypeList(GuestIpAddressTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestIpAddressTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestIpAddressList(GuestIpAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestIpAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestIpAddress(GuestIpAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestIpAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestNetworkInterfaceList(GuestNetworkInterfaceList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestNetworkInterfaceList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestNetworkInterface(GuestNetworkInterface *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestNetworkInterface(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestLogicalProcessorList(GuestLogicalProcessorList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestLogicalProcessorList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestLogicalProcessor(GuestLogicalProcessor *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestLogicalProcessor(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestDiskBusTypeList(GuestDiskBusTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestDiskBusTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestPCIAddressList(GuestPCIAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestPCIAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestPCIAddress(GuestPCIAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestPCIAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestDiskAddressList(GuestDiskAddressList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestDiskAddressList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestDiskAddress(GuestDiskAddress *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestDiskAddress(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFilesystemInfoList(GuestFilesystemInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFilesystemInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestFilesystemInfo(GuestFilesystemInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestFilesystemInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockList(GuestMemoryBlockList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlock(GuestMemoryBlock *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlock(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockResponseTypeList(GuestMemoryBlockResponseTypeList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockResponseTypeList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockResponseList(GuestMemoryBlockResponseList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockResponseList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockResponse(GuestMemoryBlockResponse *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockResponse(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockInfoList(GuestMemoryBlockInfoList *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockInfoList(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}


void qapi_free_GuestMemoryBlockInfo(GuestMemoryBlockInfo *obj)
{
    QapiDeallocVisitor *md;
    Visitor *v;

    if (!obj) {
        return;
    }

    md = qapi_dealloc_visitor_new();
    v = qapi_dealloc_get_visitor(md);
    visit_type_GuestMemoryBlockInfo(v, &obj, NULL, NULL);
    qapi_dealloc_visitor_cleanup(md);
}

